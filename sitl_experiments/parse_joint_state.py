#!/usr/bin/env python3
import re
import numpy as np
import pickle
import sys

PATH = sys.argv[1] if len(sys.argv) > 1 else '/tmp/claude-1000/-home-omer/2ebc1a5b-f0c9-4937-8214-37f5d48fdea3/scratchpad/joint_capture.txt'

JOINTS = ['motor_0_joint','motor_1_joint','motor_2_joint','rotor_0_joint','rotor_1_joint','rotor_2_joint']

t_list = []
data = {j: {'position': [], 'velocity': []} for j in JOINTS}

with open(PATH, 'r', errors='ignore') as f:
    content = f.read()

# split into messages on "header {" boundaries
msgs = content.split('header {')[1:]  # drop preamble before first message
print(f"n messages = {len(msgs)}")

sec_re = re.compile(r'sec:\s*(-?\d+)')
nsec_re = re.compile(r'nsec:\s*(-?\d+)')
joint_block_re = re.compile(r'joint\s*\{(.*?)\n\}', re.S)
name_re = re.compile(r'name:\s*"([^"]+)"')
pos_re = re.compile(r'position:\s*(-?[\d.eE+-]+)')
vel_re = re.compile(r'velocity:\s*(-?[\d.eE+-]+)')

for m in msgs:
    header_part = m.split('\n}')[0]
    sm = sec_re.search(header_part)
    nm = nsec_re.search(header_part)
    if not sm or not nm:
        continue
    t = int(sm.group(1)) + int(nm.group(1))/1e9

    joints_found = {}
    for jb in joint_block_re.finditer(m):
        block = jb.group(1)
        nmatch = name_re.search(block)
        if not nmatch:
            continue
        jname = nmatch.group(1)
        if jname not in JOINTS:
            continue
        # position/velocity appear inside axis1{...} within this block
        pmatch = pos_re.search(block)
        vmatch = vel_re.search(block)
        pos = float(pmatch.group(1)) if pmatch else np.nan
        vel = float(vmatch.group(1)) if vmatch else np.nan
        joints_found[jname] = (pos, vel)

    if len(joints_found) == 6:
        t_list.append(t)
        for j in JOINTS:
            data[j]['position'].append(joints_found[j][0])
            data[j]['velocity'].append(joints_found[j][1])

t_arr = np.array(t_list)
print(f"n complete samples = {len(t_arr)}, t range = {t_arr[0]:.2f} to {t_arr[-1]:.2f}")

out = {'t': t_arr}
for j in JOINTS:
    out[j+'_pos'] = np.array(data[j]['position'])
    out[j+'_vel'] = np.array(data[j]['velocity'])

with open('/tmp/claude-1000/-home-omer/2ebc1a5b-f0c9-4937-8214-37f5d48fdea3/scratchpad/joint_state_parsed.pkl', 'wb') as f:
    pickle.dump(out, f)

print("saved joint_state_parsed.pkl")
# quick sanity print
for j in JOINTS:
    print(j, 'pos range', out[j+'_pos'].min(), out[j+'_pos'].max(), 'vel range', out[j+'_vel'].min(), out[j+'_vel'].max())
