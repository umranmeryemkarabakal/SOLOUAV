# Asama 2: 11 hareketli parca icin revolute eklem kur.
#
# Menteşe noktalari, eksenleri ve limitleri BURAYA YAZILMAZ -- model.sdf'ten
# okunur. Bu depo daha once ayni sayinin uc yerde ayrismasindan zarar gordu;
# eklem tablosunu dorduncu bir kopya olarak tutmak ayni hatayi tekrarlamak olur.
#
# ONKOSUL: fusion_01_import.py calistirilmis olmali (23 bilesen + base_link).
#
# KULLANIM: Fusion 360 > Utilities > Scripts and Add-Ins > Scripts > + > bu dosya
import os
import re
import math
import adsk.core
import adsk.fusion
import traceback

# model.sdf yolu. Bos birakilirsa asagidaki adaylar sirayla denenir.
SDF = ""

SDF_CANDIDATES = [
    os.environ.get("TILTROTOR_SDF", ""),
    os.path.join(os.path.expanduser("~"), "Documents", "tiltrotor_cad",
                 "tiltrotor_tailplane_model.sdf"),
    os.path.join(os.path.expanduser("~"), "tiltrotor_project_updates",
                 "tiltrotor_tailplane_model.sdf"),
    os.path.join(os.path.expanduser("~"), "SOLOUAV", "tiltrotor_tailplane_model.sdf"),
]

# SDF child link adi -> CAD parca adi (cad/dogrula.py child_map ile ayni)
CHILD_TO_PART = {
    "motor_0": "motor_0_right", "motor_1": "motor_1_left", "motor_2": "motor_2_tail",
    "rotor_0": "rotor_0_right", "rotor_1": "rotor_1_left", "rotor_2": "rotor_2_tail",
    "left_elevon": "elevon_left", "right_elevon": "elevon_right",
    "left_elevator": "elevator_left", "right_elevator": "elevator_right",
    "rudder": "rudder",
}

# Eklemin bagli oldugu sabit/ust parca. SDF'te ucunun parent'i base_link, ama
# Fusion'da base_link bir Rigid Group -- bilesen degil. Bu yuzden fiziksel
# olarak komsu olan uyeye baglaniyor; rigid group sayesinde kinematik ayni.
PARENT_PART = {
    "motor_0_right": "pylon_right", "motor_1_left": "pylon_left",
    "motor_2_tail": "tail_boom",
    "rotor_0_right": "motor_0_right", "rotor_1_left": "motor_1_left",
    "rotor_2_tail": "motor_2_tail",
    "elevon_left": "wing", "elevon_right": "wing",
    "elevator_left": "tailplane", "elevator_right": "tailplane",
    "rudder": "vertical_stabiliser",
}

MM_TO_CM = 0.1          # Fusion API ic birimi cm; STEP mm olarak uretildi
CONTINUOUS = math.radians(1e6)   # bunun ustundeki limit "sinirsiz" demektir


def find_sdf():
    for path in ([SDF] if SDF else []) + SDF_CANDIDATES:
        if path and os.path.isfile(path):
            return path
    return None


def parse_sdf_joints(path):
    """model.sdf'ten menteşe noktasi, ekseni ve limitini cikar.

    cad/dogrula.py check_hinges() ile AYNI hesap:
      nokta = cocuk link pose'u + eklem pose'u   (model cercevesinde, m -> mm)
      eksen = <xyz>, use_parent_model_frame yoksa eklem pose'unun yaw'i ile dondurulur
    """
    s = open(path).read()

    # DIKKAT: bu SDF'te link adlari CIFT tirnakli, eklem adlari TEK tirnakli.
    # Iki stili de kabul et -- yalnizca birini yakalayan bir regex, link pose'u
    # sifir olmayan parcalarda (elevator, rudder) menteşeyi 790-870 mm kaydirir
    # ve bu sessizce olur.
    links = {}
    for lm in re.finditer(r"<link name=['\"]([^'\"]+)['\"]>(.*?)</link>", s, re.S):
        body = lm.group(2)
        # <inertial> icindeki pose KUTLE MERKEZIDIR, linkin yerlesimi degil.
        # Ayiklanmazsa elevon menteşesi tam 300 mm kayar (SDF'te 0 0.3 0).
        # Ayni tuzak cad/dogrula.py parse_sdf() icinde de yazili.
        head = re.sub(r"<inertial>.*?</inertial>", "", body, flags=re.S)
        if "<visual" in head:
            head = head[:head.find("<visual")]
        pm = re.search(r"<pose>([^<]+)</pose>", head)
        links[lm.group(1)] = [float(v) for v in pm.group(1).split()] if pm else [0.0] * 6

    out = {}
    for jm in re.finditer(
            r"<joint name=['\"]([^'\"]+)['\"] type=['\"]revolute['\"]>(.*?)</joint>",
            s, re.S):
        jn, body = jm.group(1), jm.group(2)
        child = re.search(r"<child>([^<]+)</child>", body).group(1)
        part = CHILD_TO_PART.get(child)
        if part is None:
            continue

        pm = re.search(r"<pose>([^<]+)</pose>", body)
        jpose = [float(v) for v in pm.group(1).split()] if pm else [0.0] * 6
        cpose = links.get(child, [0.0] * 6)
        point_mm = [(cpose[i] + jpose[i]) * 1000.0 for i in range(3)]

        axis = [float(v) for v in re.search(r"<xyz>([^<]+)</xyz>", body).group(1).split()]
        if "<use_parent_model_frame>1" not in body:
            yaw = jpose[5]
            c, sn = math.cos(yaw), math.sin(yaw)
            axis = [c * axis[0] - sn * axis[1], sn * axis[0] + c * axis[1], axis[2]]

        lower = float(re.search(r"<lower>([^<]+)</lower>", body).group(1))
        upper = float(re.search(r"<upper>([^<]+)</upper>", body).group(1))

        out[part] = dict(joint=jn, point=point_mm, axis=axis,
                         lower=lower, upper=upper,
                         continuous=(upper > CONTINUOUS))
    return out


def _nokta_ekle(comp, point, name):
    """Bilesenin icine, verilen konumda adlandirilmis bir construction point."""
    cp_in = comp.constructionPoints.createInput()
    cp_in.setByPoint(point)
    cp = comp.constructionPoints.add(cp_in)
    cp.name = name
    return cp


def find_occurrence(root, name):
    for occ in root.occurrences:
        if occ.component.name == name:
            return occ
    return None


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        sdf_path = find_sdf()
        if sdf_path is None:
            ui.messageBox("model.sdf bulunamadi.\n\nBu dosyanin basindaki SDF degiskenine\n"
                          "tam yolu yazin.\n\nDenenen yollar:\n  "
                          + "\n  ".join(p for p in SDF_CANDIDATES if p))
            return

        table = parse_sdf_joints(sdf_path)
        if len(table) != 11:
            ui.messageBox("SDF'ten 11 eklem beklendi, {} bulundu.\n\nDosya:\n{}"
                          .format(len(table), sdf_path))
            return

        design = adsk.fusion.Design.cast(
            app.activeDocument.products.itemByProductType("DesignProductType"))
        if design is None:
            ui.messageBox("Aktif bir Design dosyasi yok.")
            return
        root = design.rootComponent

        eksik = [n for n in list(table) + list(set(PARENT_PART.values()))
                 if find_occurrence(root, n) is None]
        if eksik:
            ui.messageBox("Su bilesenler yok: {}\n\nOnce fusion_01_import.py'yi calistirin."
                          .format(", ".join(sorted(set(eksik)))))
            return

        if root.joints.count:
            ui.messageBox("Bu Design'da zaten {} eklem var.\n\nIkinci kez calistirmak kopya "
                          "uretir; once mevcut eklemleri silin.".format(root.joints.count))
            return

        kurulan = []
        for part in sorted(table):
            J = table[part]
            child_occ = find_occurrence(root, part)
            parent_occ = find_occurrence(root, PARENT_PART[part])
            child_comp = child_occ.component

            px, py, pz = [v * MM_TO_CM for v in J["point"]]
            pivot = adsk.core.Point3D.create(px, py, pz)
            direction = adsk.core.Vector3D.create(*J["axis"])

            # Menteşe noktasi ve ekseni cocuk bilesenin icinde insa edilir.
            # Occurrence donusumu birim matris oldugu icin bilesen cercevesi
            # kok cerceveyle ayni -- SDF koordinatlari dogrudan kullanilabilir.
            # Ayni dunya noktasi HER IKI bilesende de insa edilir. Parcalar zaten
            # dogru konumda oldugu icin iki nokta ust uste gelir ve eklem
            # kurulurken hicbir sey yerinden oynamaz.
            cp_child = _nokta_ekle(child_comp, pivot, J["joint"] + "_nokta")
            cp_parent = _nokta_ekle(parent_occ.component, pivot, J["joint"] + "_nokta")

            ax_in = child_comp.constructionAxes.createInput()
            ax_in.setByLine(adsk.core.InfiniteLine3D.create(pivot, direction))
            ax = child_comp.constructionAxes.add(ax_in)
            ax.name = J["joint"] + "_eksen"

            # Geometri proxy'den kuruluyor; baglam boylece tasindigi icin
            # occurrenceOne/Two AYRICA verilmez (ikisi birlikte cakisir).
            j_in = root.joints.createInput(
                adsk.fusion.JointGeometry.createByPoint(
                    cp_child.createForAssemblyContext(child_occ)),
                adsk.fusion.JointGeometry.createByPoint(
                    cp_parent.createForAssemblyContext(parent_occ)))
            j_in.setAsRevoluteJointMotion(
                adsk.fusion.JointDirections.CustomJointDirection,
                ax.createForAssemblyContext(child_occ))
            joint = root.joints.add(j_in)
            joint.name = J["joint"]

            if not J["continuous"]:
                lim = joint.jointMotion.rotationLimits
                lim.isMinimumValueEnabled = True
                lim.minimumValue = J["lower"]
                lim.isMaximumValueEnabled = True
                lim.maximumValue = J["upper"]
                kurulan.append("{}  [{:+.1f}, {:+.1f}] derece"
                               .format(J["joint"], math.degrees(J["lower"]),
                                       math.degrees(J["upper"])))
            else:
                kurulan.append("{}  serbest (pervane)".format(J["joint"]))

        ui.messageBox("TAMAM. {} eklem kuruldu.\n\nKaynak:\n{}\n\n{}\n\n"
                      "Kuyruk tilt limiti SDF'ten geldigi icin dogrudur (0...20 derece)."
                      .format(len(kurulan), sdf_path, "\n".join(kurulan)))

    except:  # noqa: E722 -- Fusion script idiyomu
        if ui:
            ui.messageBox("Script hata verdi:\n{}".format(traceback.format_exc()))
        else:
            raise


# Fusion Scripts panelinde run() Fusion tarafindan cagrilir; kopru uzerinden
# (fusion_execute) duz exec edilirse cagiran olmaz -- bkz. fusion_01_import.py.
if globals().get("__name__") in (None, "__main__"):
    run(None)
