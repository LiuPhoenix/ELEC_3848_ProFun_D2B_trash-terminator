#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Manual Motion Test (no AI model)
=================================
Test the full sorting sequence without object detection.
The user manually specifies the trash type (metal / plastic)
and places the trash at the designated position.

Sequence (executed by firmware via SEQ command):
  Phase 1 -- Positioning & Pickup
    1. FRONTPID:25  -> ultrasonic step to 25cm from wall A
    2. WALIGN      -> ultrasonic wall alignment
    3. WAIT:CAM    -> pause, manually place trash, press Enter
    4. RELEASE     -> open gripper
    5. FRONTPID:6  -> ultrasonic step to 6cm (pickup distance)
    6. GRAB        -> close gripper
  Phase 2 -- Transport to Drop Zone
    7. FRONTPID:30  -> ultrasonic step back to 30cm
    8. ROT:180      -> timed 180-degree rotation (no gyro)
    9. Arm to 45deg -> lower arm for drop-off
   10. FRONTPID:5   -> ultrasonic step forward to 5cm from wall B
  Phase 3 -- Drop & Return (by type)
    Metal:  strafe left 7cm -> release -> back 15cm -> strafe right 7cm -> reset arm -> align -> rotate -> align -> 25cm
    Plastic: strafe right 7cm -> release -> back 15cm -> strafe left 7cm -> reset arm -> align -> rotate -> align -> 25cm

Usage: python manual_test.py
Prerequisites:
  - Firmware flashed (firmware/trash_robot_firmware.ino)
  - robot_serial.py in the same directory
  - calibration.json and grip_angles.json configured
"""

import time
import json
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from robot_serial import RobotSerial, find_arduino_port

CALIB_FILE = os.path.join(os.path.dirname(__file__), 'config', 'calibration.json')
GRIP_FILE = os.path.join(os.path.dirname(__file__), 'config', 'grip_angles.json')


def load_params():
    """Load calibration parameters from config files."""
    params = {}
    if os.path.exists(CALIB_FILE):
        with open(CALIB_FILE, 'r') as f:
            params.update(json.load(f))
    if os.path.exists(GRIP_FILE):
        with open(GRIP_FILE, 'r') as f:
            grip = json.load(f)
            params['GRIP_OPEN'] = grip.get('grip_open', 30)
            params['GRIP_CLOSE'] = grip.get('grip_close', 90)
    return params


def upload_all_params(bot, params):
    """Upload all parameters to Arduino."""
    print('=== Uploading params to Arduino ===')
    for key, value in params.items():
        result = bot.set_param(key, value)
        if result:
            print('  {} = {}'.format(key, value))
    print('[OK] Params uploaded')


# ================================================================
# WAIT callback (manual mode)
# ================================================================
def on_wait_manual(event_type, info):
    """
    Manual WAIT callback for testing.

    event_type:
      'CAM'  -> camera/recognition phase (place trash manually)
      'STEP' -> step-by-step advance (legacy)
      other  -> auto-continue

    Returns True to continue, False to abort.
    """
    if event_type == 'CAM':
        print('')
        print('  ========================================')
        print('  [WAIT:CAM] Recognition phase')
        print('  Manual mode: place trash in front of gripper')
        print('  (Firmware has completed FRONTPID:25 + WALIGN)')
        print('  (Next auto: open gripper -> advance to 6cm -> grab)')
        print('  ========================================')

        choice = input('  Enter CONTINUE when ready, STOP to abort: ').strip().upper()
        return choice != 'STOP'

    elif event_type == 'STEP':
        print('\n[Step {}] Waiting for confirmation...'.format(info))
        choice = input('  Enter CONTINUE to proceed, STOP to abort: ').strip().upper()
        return choice != 'STOP'

    else:
        print('\n[WAIT:{}] Unknown event, auto-continue'.format(event_type))
        return True


# ================================================================
# Test Functions
# ================================================================
def test_single_grab(bot):
    """Test single grab action."""
    print('\n=== Single Grab Test ===')
    input('Place trash in front of gripper, press Enter...')
    bot.grab()
    time.sleep(1)
    print('Grab done')


def test_single_release(bot):
    """Test single release action."""
    print('\n=== Single Release Test ===')
    bot.release()
    time.sleep(0.5)
    print('Release done')


def test_sequence(bot, trash_type):
    """Test full sorting sequence for a given trash type."""
    print('\n=== {} Sorting Path ==='.format(trash_type.upper()))

    if trash_type == 'metal':
        print('Phase 1: FRONTPID:25 -> WALIGN -> [place trash] -> RELEASE -> FRONTPID:6 -> GRAB')
        print('Phase 2: FRONTPID:30 -> ROT:180 -> ARM45 -> FRONTPID:5')
        print('Phase 3: SL7 -> RELEASE -> FRONTPID:15 -> SR7 -> ARM_HOME -> WALIGN -> ROT:180 -> WALIGN -> FRONTPID:25')
    elif trash_type == 'plastic':
        print('Phase 1: FRONTPID:25 -> WALIGN -> [place trash] -> RELEASE -> FRONTPID:6 -> GRAB')
        print('Phase 2: FRONTPID:30 -> ROT:180 -> ARM45 -> FRONTPID:5')
        print('Phase 3: SR7 -> RELEASE -> FRONTPID:15 -> SL7 -> ARM_HOME -> WALIGN -> ROT:180 -> WALIGN -> FRONTPID:25')
    else:
        print('[ERR] Unsupported type: {} (only metal / plastic)'.format(trash_type))
        return

    input('\nPress Enter to start sequence...')
    success = bot.run_sequence(trash_type, on_wait=on_wait_manual)
    if success:
        print('[OK] Sequence completed! (back to start position)')
    else:
        print('[FAIL] Sequence failed or aborted')


def main():
    print('=' * 60)
    print('Manual Motion Test (ultrasonic stepping, no gyro)')
    print('Supported: metal / plastic')
    print('=' * 60)

    params = load_params()
    print('\nLoaded params:')
    for k, v in params.items():
        print('  {} = {}'.format(k, v))

    # Connect
    port = find_arduino_port()
    if not port:
        print('[ERR] Arduino not found')
        sys.exit(1)

    bot = RobotSerial(port)
    bot.set_debug(True)

    if not bot.connect():
        sys.exit(1)

    # Upload params
    upload_all_params(bot, params)

    try:
        while True:
            print('\n' + '=' * 60)
            print('Menu:')
            print('  1. Single grab test')
            print('  2. Single release test')
            print('  --- Sorting Sequences (firmware auto-exec) ---')
            print('  3. Metal sorting path (SEQ:METAL)')
            print('  4. Plastic sorting path (SEQ:PLASTIC)')
            print('  --- Tools ---')
            print('  5. Send raw command')
            print('  6. Re-upload params')
            print('  7. Read distances')
            print('  0. Exit')
            print('=' * 60)

            choice = input('Select: ').strip()

            if choice == '0':
                break

            elif choice == '1':
                test_single_grab(bot)

            elif choice == '2':
                test_single_release(bot)

            elif choice == '3':
                test_sequence(bot, 'metal')

            elif choice == '4':
                test_sequence(bot, 'plastic')

            elif choice == '5':
                cmd = input('Command: ').strip()
                if cmd:
                    bot._send(cmd)
                    time.sleep(2)

            elif choice == '6':
                upload_all_params(bot, params)

            elif choice == '7':
                d = bot.get_distances()
                print('FL={} FR={} SL={} SR={}'.format(
                    d['fl'], d['fr'], d['sl'], d['sr']))

    except KeyboardInterrupt:
        print('\n[Interrupted]')
        bot.stop()

    finally:
        bot.disconnect()


if __name__ == '__main__':
    main()
