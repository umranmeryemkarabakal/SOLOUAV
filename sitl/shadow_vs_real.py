#!/usr/bin/env python3
"""shadow_vs_real.py — mc_indi_tiltrotor GOLGE aktuator modeli vs GERCEK Gazebo eklemi.

NEDEN (Adim 18, 2026-07-28):
`MulticopterIndiTiltrotor.cpp` icindeki `_u_actual` **acik cevrim** bir golge
modeldir (1. derece gecikme + slew clamp, Gazebo'dan SIFIR geri besleme). Gercek
tilt servosu ise `JointPositionController`: P=100, I=D=0, `cmd_max=2` (tork
limiti), `err_max=0.2` -> |hata| >= 0.02 rad icin sabit 2 Nm ile doyan,
TORK-SINIRLI 2. DERECE bir sistem. INDI'nin lineerlestirme noktasi ve WLS'in G
matrisi bu `_u_actual`'dan turedigi icin sapma dogrudan kontrol yasasini bozar.

KULLANIM
--------
1) PX4 logger'a gerekli topic'leri ekleyin (REBUILD GEREKMEZ):
     mkdir -p ~/PX4-Autopilot/build/px4_sitl_default/rootfs/etc/logging
     cp sitl/logger_topics_shadow.txt \\
        ~/PX4-Autopilot/build/px4_sitl_default/rootfs/etc/logging/logger_topics.txt
   !! Bu dosya varsa PX4 logger'i VARSAYILAN profili TAMAMEN degistirir
      (logged_topics.cpp:560 if/else). Olcum bitince MUTLAKA SILIN.

2) Modelde JointStatePublisher eklentisi olmali (kalici olarak eklendi):
     Tools/simulation/gz/models/tiltrotor_indi/model.sdf sonunda,
     motor_{0,1,2}_joint icin. Yalnizca gozlem; fizigi/kontrolu etkilemez.

3) SITL'i baslatin, sonra ucustan ONCE gercek eklem kaydini baslatin:
     sitl/gz_joint_csv.sh > /tmp/gz_joint.csv &

4) Ucusu kosun (bkz. .claude/skills/sitl-lockup-check). Dusuk hiz icin
   BASTAN pitch trim verin: test_sp 0 0.061 <yaw_sp> 0 <z_sp> 1 1 0

5) Analiz:
     python3 sitl/shadow_vs_real.py <ulog.ulg> /tmp/gz_joint.csv [cikti.png]

DIKKAT — OLCUM TUZAGI (Adim 18'de yasandi): `tiltrotor_indi_status`
ulog'unda ~1.5% oraninda YINELENEN zaman damgasi var (dt = 0). Elenmezse
interpolasyon 10-12 derecelik sahte sapma sivrileri uretir. Bu script onlari
eler ve istatistiklerde `max` yaninda **p99**'u da basar — p99'a bakin.
"""
import sys
import numpy as np
from pyulog import ULog

TOPICS = ['tiltrotor_indi_status', 'tiltrotor_indi_setpoint', 'vehicle_angular_velocity',
          'vehicle_attitude', 'vehicle_local_position']


def dedup(t, *arrays):
    """Yinelenen/geri giden zaman damgalarini ele (bkz. yukaridaki OLCUM TUZAGI)."""
    keep = np.concatenate([[True], np.diff(t) > 1e-6])
    return (t[keep],) + tuple(a[keep] for a in arrays)


def load(ulg_path, gz_path):
    u = ULog(ulg_path, TOPICS)
    D = {d.name: d for d in u.data_list}
    missing = [k for k in TOPICS if k not in D]
    if missing:
        raise SystemExit(f'ulog eksik topic: {missing}\n'
                         '-> logger_topics_shadow.txt kopyalanmadan mi kosuldu?')

    st = D['tiltrotor_indi_status']
    t = st.data['timestamp'] / 1e6
    sh = np.column_stack([st.data[f'u_actual[{3 + i}]'] for i in range(3)])
    t, sh = dedup(t, sh)

    gz = np.loadtxt(gz_path, delimiter=',')
    gz = gz[np.argsort(gz[:, 0])]
    gz_t, gz_d = dedup(gz[:, 0], gz[:, 1:4])

    sp = D['tiltrotor_indi_setpoint']
    sp_t, sp_y = sp.data['timestamp'] / 1e6, sp.data['yaw_sp']
    j = np.where(np.abs(np.diff(sp_y)) > 0.4)[0]
    t_step = sp_t[j[0] + 1] if len(j) else sp_t[0]

    av = D['vehicle_angular_velocity']
    at = D['vehicle_attitude']
    q = np.column_stack([at.data[f'q[{i}]'] for i in range(4)])
    lp = D['vehicle_local_position']
    return dict(
        t=t, sh=sh, gz_t=gz_t, gz_d=gz_d, t_step=t_step,
        av_t=av.data['timestamp'] / 1e6, r=av.data['xyz[2]'],
        at_t=at.data['timestamp'] / 1e6,
        yaw=np.degrees(np.arctan2(2 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]),
                                  1 - 2 * (q[:, 2] ** 2 + q[:, 3] ** 2))),
        lp_t=lp.data['timestamp'] / 1e6,
        vh=np.hypot(lp.data['vx'], lp.data['vy']))


def report(d):
    t, sh, gz_t, gz_d, t_step = d['t'], d['sh'], d['gz_t'], d['gz_d'], d['t_step']
    lo, hi = max(t[0], gz_t[0]), min(t[-1], gz_t[-1])
    grid = np.arange(lo, hi, 0.004)
    shi = np.column_stack([np.interp(grid, t, sh[:, i]) for i in range(3)])
    rei = np.column_stack([np.interp(grid, gz_t, gz_d[:, i]) for i in range(3)])
    err = np.degrees(shi - rei)
    print(f'ortak pencere {lo:.1f}..{hi:.1f} s, yaw adimi t={t_step:.2f} s\n')

    def blk(title, m):
        print(f'--- {title}  (n={m.sum()}) ---')
        print(f'{"":5s} {"ort":>8s} {"RMS":>8s} {"p99":>8s} {"max*":>8s} '
              f'{"golge ort":>10s} {"gercek ort":>11s}')
        for i in range(3):
            e = err[m, i]
            print(f'  d{i}:{e.mean():7.3f}d {np.sqrt((e ** 2).mean()):7.3f}d '
                  f'{np.percentile(np.abs(e), 99):7.3f}d {np.abs(e).max():7.2f}d '
                  f'{np.degrees(shi[m, i]).mean():9.3f}d {np.degrees(rei[m, i]).mean():10.3f}d')
        print()

    blk('TUM PENCERE', grid > grid[0] + 2)
    blk('YAW ADIMI ONCESI', (grid > t_step - 20) & (grid < t_step))
    blk('YAW ADIMI SONRASI', (grid > t_step + 10) & (grid < t_step + 22))
    print('* max, kalan ornekleme artefaktlarini tasiyabilir — p99 sutununa bakin.\n')

    # sinir etkisi: golge tam 0 derken gercek nerede?
    m = grid > grid[0] + 2
    for i in (1, 2):
        onb = np.degrees(shi[m, i]) < 0.05
        if onb.sum():
            print(f'golge d{i}, orneklerin %{100 * onb.mean():.0f}\'inde TILT_MIN=0 sinirinda '
                  f'(WLS: "asagi inemez"); ayni anlarda GERCEK d{i} ort = '
                  f'{np.degrees(rei[m][onb, i]).mean():.3f} deg -> o kadar hareket alani '
                  f'ALLOCATOR\'DAN GIZLENIYOR.')

    # yaw davranisi
    av_t, r, at_t, yaw, lp_t, vh = d['av_t'], d['r'], d['at_t'], d['yaw'], d['lp_t'], d['vh']
    print('\n=== YAW ===')
    for a, b, lbl in [(max(t[0], t_step - 20), t_step, 'adim oncesi 20 s'),
                      (t_step + 10, t_step + 22, 'adim + 10..22 s')]:
        mv, ma, ml = (av_t >= a) & (av_t < b), (at_t >= a) & (at_t < b), (lp_t >= a) & (lp_t < b)
        if mv.sum() == 0:
            continue
        print(f'{lbl:18s} yaw {yaw[ma].min():7.1f}..{yaw[ma].max():6.1f} deg '
              f'(band {yaw[ma].max() - yaw[ma].min():5.1f})  r RMS {np.sqrt((r[mv] ** 2).mean()):.4f} '
              f'max|r| {np.abs(r[mv]).max():.3f}  vh {vh[ml].mean():.2f} m/s')
    return grid, shi, rei


def plot(d, grid, shi, rei, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t_step = d['t_step']
    T0, T1 = grid[0], min(grid[-1], t_step + 24)

    def w(x, y):
        m = (x >= T0) & (x <= T1)
        return x[m] - t_step, y[m]

    fig, ax = plt.subplots(4, 1, figsize=(13, 11), sharex=True)
    ax[0].plot(*w(d['at_t'], d['yaw']), 'C3', lw=1.4)
    ax[0].axhline(0, ls='--', c='gray', lw=.9)
    ax[0].axvline(0, ls=':', c='k', lw=1.2)
    ax[0].set_ylabel('yaw (deg)'); ax[0].grid(alpha=.3)
    ax[0].set_title('yaw acisi (dikey nokta cizgi = yaw setpoint degisimi)')

    ax[1].plot(*w(d['av_t'], d['r']), 'C0', lw=1.1)
    for s in (0.5, -0.5):
        ax[1].axhline(s, ls=':', c='k', lw=.8)
    ax[1].axvline(0, ls=':', c='k', lw=1.2)
    ax[1].set_ylabel('yaw hizi r (rad/s)'); ax[1].grid(alpha=.3)
    ax[1].set_title('yaw hizi (nokta cizgi: RATE_SP_LIMIT_yaw = 0.5 rad/s)')

    m = (grid >= T0) & (grid <= T1)
    for i, c in zip(range(3), ['C0', 'C1', 'C2']):
        ax[2].plot(grid[m] - t_step, np.degrees(shi[m, i]), c, lw=1.2, label=f'golge $\\delta_{i}$')
        ax[2].plot(grid[m] - t_step, np.degrees(rei[m, i]), c, lw=1.0, ls='--',
                   label=f'gercek $\\delta_{i}$')
    ax[2].axhline(0, c='k', lw=1.4, alpha=.6)
    ax[2].axvline(0, ls=':', c='k', lw=1.2)
    ax[2].set_ylabel('tilt (deg)'); ax[2].legend(loc='upper right', ncol=3, fontsize=8)
    ax[2].grid(alpha=.3)
    ax[2].set_title('Golge (_u_actual, duz) vs GERCEK Gazebo eklemi (kesikli) — '
                    'kalin siyah: TILT_MIN = 0')

    ax[3].plot(grid[m] - t_step, np.degrees(rei[m, 1]), 'C1', lw=1.3, label='gercek $\\delta_1$')
    ax[3].plot(grid[m] - t_step, np.degrees(shi[m, 1]), 'C1', lw=1.0, ls='--', label='golge $\\delta_1$')
    ax[3].axhline(0, c='k', lw=1.4, alpha=.6, label='TILT_MIN = 0')
    ax[3].axvline(0, ls=':', c='k', lw=1.2)
    ax[3].set_ylabel('$\\delta_1$ (deg)'); ax[3].set_xlabel('yaw setpoint degisiminden itibaren (s)')
    ax[3].set_ylim(-0.3, 3.5); ax[3].legend(loc='upper right'); ax[3].grid(alpha=.3)
    ax[3].set_title('YAKINLASTIRMA $\\delta_1$ — sinirdan kalkis/cakilma cevrimleri')

    plt.tight_layout()
    plt.savefig(out, dpi=105)
    print(f'\n-> {out}')


if __name__ == '__main__':
    if len(sys.argv) < 3:
        raise SystemExit(__doc__)
    data = load(sys.argv[1], sys.argv[2])
    g, s, r = report(data)
    plot(data, g, s, r, sys.argv[3] if len(sys.argv) > 3 else 'shadow_vs_real_tilt.png')
