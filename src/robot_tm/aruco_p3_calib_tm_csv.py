#!/usr/bin/env python3
import csv
import json
import time
from pathlib import Path
from robot_command import Robot

ROBOT_IP = '192.168.10.3'
CSV_PATH = '/home/hucenrotia/mahfud/Calibration_TM/src/robot_tm/tm_pose_log.csv'
STATE_JSON_PATH = '/home/hucenrotia/mahfud/Calibration_TM/src/robot_tm/tm_runtime_state.json'
LOG_INTERVAL_SEC = 0.05

FIELDNAMES = [
    'timestamp',
    'robot_x', 'robot_y', 'robot_z', 'robot_rx', 'robot_ry', 'robot_rz',
    'selected_point', 'marker_detected',
    'marker_base_x', 'marker_base_y', 'marker_base_z', 'marker_base_rx', 'marker_base_ry', 'marker_base_rz',
    'p1_contact_x', 'p1_contact_y', 'p1_contact_z', 'p1_contact_rx', 'p1_contact_ry', 'p1_contact_rz',
    'p1_approach_x', 'p1_approach_y', 'p1_approach_z', 'p1_approach_rx', 'p1_approach_ry', 'p1_approach_rz',
    'p2_contact_x', 'p2_contact_y', 'p2_contact_z', 'p2_contact_rx', 'p2_contact_ry', 'p2_contact_rz',
    'p2_approach_x', 'p2_approach_y', 'p2_approach_z', 'p2_approach_rx', 'p2_approach_ry', 'p2_approach_rz',
    'p3_contact_x', 'p3_contact_y', 'p3_contact_z', 'p3_contact_rx', 'p3_contact_ry', 'p3_contact_rz',
    'p3_approach_x', 'p3_approach_y', 'p3_approach_z', 'p3_approach_rx', 'p3_approach_ry', 'p3_approach_rz',
]

def pose_to_cols(prefix, pose):
    row = {}
    names = ['x', 'y', 'z', 'rx', 'ry', 'rz']
    if not pose:
        for n in names:
            row[f'{prefix}_{n}'] = ''
        return row
    for n, v in zip(names, pose[:6]):
        row[f'{prefix}_{n}'] = v
    return row

def read_state_json():
    path = Path(STATE_JSON_PATH)
    if not path.exists():
        return {}
    try:
        with path.open('r') as f:
            return json.load(f)
    except Exception:
        return {}

def ensure_csv():
    path = Path(CSV_PATH)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists() or path.stat().st_size == 0:
        with path.open('w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
            writer.writeheader()

def append_row(row):
    with open(CSV_PATH, 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
        writer.writerow(row)

def main():
    ensure_csv()
    robot = Robot(robot_type='TM', ip_address=ROBOT_IP, debug=False)

    while True:
        status, pos = robot.get_position(retries=1, retry_delay=0.02)
        state = read_state_json()

        row = {
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S') + f'.{int((time.time()%1)*1000):03d}',
            'selected_point': state.get('selected_point', ''),
            'marker_detected': int(bool(state.get('marker_detected', False))),
        }

        if status == 0 and pos is not None:
            row.update({
                'robot_x': pos.get('x', ''),
                'robot_y': pos.get('y', ''),
                'robot_z': pos.get('z', ''),
                'robot_rx': pos.get('rx', ''),
                'robot_ry': pos.get('ry', ''),
                'robot_rz': pos.get('rz', ''),
            })
        else:
            row.update({
                'robot_x': '', 'robot_y': '', 'robot_z': '',
                'robot_rx': '', 'robot_ry': '', 'robot_rz': '',
            })

        row.update(pose_to_cols('marker_base', state.get('marker_base')))
        row.update(pose_to_cols('p1_contact', state.get('p1_contact')))
        row.update(pose_to_cols('p1_approach', state.get('p1_approach')))
        row.update(pose_to_cols('p2_contact', state.get('p2_contact')))
        row.update(pose_to_cols('p2_approach', state.get('p2_approach')))
        row.update(pose_to_cols('p3_contact', state.get('p3_contact')))
        row.update(pose_to_cols('p3_approach', state.get('p3_approach')))

        append_row(row)
        time.sleep(LOG_INTERVAL_SEC)

if __name__ == '__main__':
    main()