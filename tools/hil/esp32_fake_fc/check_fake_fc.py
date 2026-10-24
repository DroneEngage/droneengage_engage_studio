#!/usr/bin/env python3
"""Bench check for esp32_fake_fc from the host side of its USB-serial.

Proves the same thing Andruav needs over OTG: heartbeats arrive, SET_MODE
GUIDED is acked MAV_RESULT_ACCEPTED, and the next HEARTBEAT carries the new
custom_mode. Exit 0 on pass.

    python3 check_fake_fc.py [--device /dev/ttyUSB0] [--baud 115200]
"""
import argparse
import sys
import time

from pymavlink import mavutil

COPTER_GUIDED = 4
COPTER_STABILIZE = 0


def wait_mode(m, mode, timeout=5):
    end = time.time() + timeout
    while time.time() < end:
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb and hb.get_srcSystem() == m.target_system and hb.custom_mode == mode:
            return True
    return False


def set_mode(m, mode):
    m.mav.set_mode_send(m.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode)
    ack = m.recv_match(type='COMMAND_ACK', blocking=True, timeout=3,
                       condition='COMMAND_ACK.command==11')
    return ack


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--device', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200)
    a = ap.parse_args()

    m = mavutil.mavlink_connection(a.device, baud=a.baud, source_system=255,
                                   source_component=190)
    ok = True
    hb = m.wait_heartbeat(timeout=10)
    if not hb:
        print('FAIL no heartbeat')
        return 1
    print(f'PASS heartbeat sys={m.target_system} type={hb.type} '
          f'autopilot={hb.autopilot} mode={m.flightmode}')

    for mode, name in ((COPTER_GUIDED, 'GUIDED'), (COPTER_STABILIZE, 'STABILIZE')):
        ack = set_mode(m, mode)
        if not ack or ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print(f'FAIL SET_MODE {name} ack={ack}')
            ok = False
            continue
        print(f'PASS SET_MODE {name} acked ACCEPTED')
        if wait_mode(m, mode):
            print(f'PASS heartbeat now reports {name}')
        else:
            print(f'FAIL heartbeat never reported {name}')
            ok = False

    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
