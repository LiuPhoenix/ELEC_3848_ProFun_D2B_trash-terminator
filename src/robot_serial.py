#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unified Robot Serial Communication Library
==========================================
Provides all commands needed for the Trash Terminator system.

Methods:
  --- Communication ---
  connect() / disconnect() / is_connected()
  send(cmd) / readline(timeout) / wait_for(prefixes, timeout)
  send_and_wait(cmd, done_prefix, timeout)
  set_debug(enable)

  --- Basic Motion ---
  forward(pwm, ms)          -> FWD:pwm:ms
  backward(pwm, ms)         -> BWD:pwm:ms
  strafe_left(pwm, ms)      -> SL:pwm:ms
  strafe_right(pwm, ms)     -> SR:pwm:ms
  stop()                    -> STOP

  --- Rotation ---
  rotate(angle)             -> ROT:angle       (timed, no gyro)
  rotate_cw(pwm, ms)        -> ROTC:pwm:ms     (timed clockwise)
  rotate_ccw(pwm, ms)       -> ROTCCW:pwm:ms   (timed counter-clockwise)

  --- Arm ---
  arm_home()                -> ARMHOME
  low_arm(angle)            -> LOWARM:angle
  grip(angle)               -> GRIP:angle
  grab()                    -> GRAB             (firmware uses GRIP_CLOSE)
  release()                 -> RELEASE          (firmware uses GRIP_OPEN)

  --- Parameters / Sensors ---
  set_param(param, value)   -> SET:param:value
  get_param(param)          -> GET:param
  get_distances()           -> DIST

  --- PID Control ---
  align_wall()              -> WALIGN           (PID wall alignment)
  front_to_distance(target) -> FRONTPID:target  (PID front distance)

  --- Photo Sync ---
  strafe_left_photo(pwm, ms, on_photo_ready)  -> SHOTL:pwm:ms
  strafe_right_photo(pwm, ms, on_photo_ready) -> SHOTR:pwm:ms

  --- Sequence ---
  run_sequence(trash_type, step_callback)      -> SEQ:type

  --- Raw Send ---
  _send(cmd)               (fire-and-forget, no reply)
"""

import glob
import serial
import sys
import time


class RobotSerial:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, timeout=1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self._debug = False

    # ================================================================
    # Communication
    # ================================================================
    def set_debug(self, enable=True):
        self._debug = enable

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            time.sleep(2.0)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            start = time.time()
            while time.time() - start < 5:
                self.send('PING')
                time.sleep(0.2)
                line = self.readline(timeout=0.5)
                if line and ('READY' in line or 'PONG' in line):
                    print('[OK] Connected: {}'.format(self.port))
                    return True

            print('[WARN] No READY received, but serial port is open: {}'.format(self.port))
            return True
        except serial.SerialException as e:
            print('[ERR] Serial connection failed: {}'.format(e))
            return False

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print('[OK] Disconnected')

    def is_connected(self):
        return self.ser is not None and self.ser.is_open

    def send(self, cmd):
        if not self.is_connected():
            raise RuntimeError('Serial not connected')
        if self._debug:
            print('[TX] {}'.format(cmd))
        self.ser.write((cmd + '\n').encode('utf-8'))

    def readline(self, timeout=None):
        if not self.is_connected():
            return None

        old_timeout = None
        if timeout is not None:
            old_timeout = self.ser.timeout
            self.ser.timeout = timeout

        try:
            line = self.ser.readline().decode('utf-8', errors='replace').strip()
        finally:
            if timeout is not None and old_timeout is not None:
                self.ser.timeout = old_timeout

        if line and self._debug:
            print('[RX] {}'.format(line))
        return line

    def wait_for(self, prefixes, timeout=30):
        if isinstance(prefixes, str):
            prefixes = (prefixes,)
        start = time.time()
        while time.time() - start < timeout:
            line = self.readline(timeout=0.2)
            if line is None or line == '':
                continue
            if any(line.startswith(prefix) for prefix in prefixes):
                return line
            if line.startswith('LOG:'):
                print('[Arduino] {}'.format(line[4:]))
            elif line.startswith('ERR:'):
                print('[Arduino ERROR] {}'.format(line[4:]))
                return None
        return None

    def send_and_wait(self, cmd, done_prefix, timeout=30):
        self.send(cmd)
        return self.wait_for(done_prefix, timeout=timeout)

    def _send(self, cmd):
        """Raw send without waiting for reply."""
        self.send(cmd)

    # ================================================================
    # Basic Motion
    # ================================================================
    def forward(self, pwm=70, ms=200):
        return self.send_and_wait('FWD:{}:{}'.format(int(pwm), int(ms)), 'DONE:FWD')

    def backward(self, pwm=70, ms=200):
        return self.send_and_wait('BWD:{}:{}'.format(int(pwm), int(ms)), 'DONE:BWD')

    def strafe_left(self, pwm=100, ms=200):
        return self.send_and_wait('SL:{}:{}'.format(int(pwm), int(ms)), 'DONE:SL')

    def strafe_right(self, pwm=100, ms=200):
        return self.send_and_wait('SR:{}:{}'.format(int(pwm), int(ms)), 'DONE:SR')

    def stop(self):
        return self.send_and_wait('STOP', 'DONE:STOP', timeout=2) is not None

    # ================================================================
    # Rotation
    # ================================================================
    def rotate(self, angle):
        """Timed rotation (no gyroscope) -> ROT:angle"""
        return self.send_and_wait('ROT:{}'.format(float(angle)), 'DONE:ROT', timeout=30)

    def rotate_cw(self, pwm=70, ms=2000):
        """Timed clockwise rotation -> ROTC:pwm:ms"""
        timeout = max(5, ms / 1000 + 3)
        return self.send_and_wait('ROTC:{}:{}'.format(int(pwm), int(ms)), 'DONE:ROTC', timeout=timeout)

    def rotate_ccw(self, pwm=70, ms=2000):
        """Timed counter-clockwise rotation -> ROTCCW:pwm:ms"""
        timeout = max(5, ms / 1000 + 3)
        return self.send_and_wait('ROTCCW:{}:{}'.format(int(pwm), int(ms)), 'DONE:ROTCCW', timeout=timeout)

    # ================================================================
    # Arm
    # ================================================================
    def arm_home(self):
        return self.send_and_wait('ARMHOME', 'DONE:ARMHOME', timeout=4) is not None

    def low_arm(self, angle):
        self.send('LOWARM:{}'.format(int(angle)))
        return self.wait_for('DATA:LOWARM:', timeout=3)

    def grip(self, angle):
        self.send('GRIP:{}'.format(int(angle)))
        return self.wait_for('DATA:GRIP:', timeout=3)

    def grab(self):
        """Grab (close gripper) -> GRAB"""
        return self.send_and_wait('GRAB', 'DONE:GRAB', timeout=3) is not None

    def release(self):
        """Release (open gripper) -> RELEASE"""
        return self.send_and_wait('RELEASE', 'DONE:RELEASE', timeout=3) is not None

    # ================================================================
    # Parameters / Sensors
    # ================================================================
    def set_param(self, param, value):
        self.send('SET:{}:{}'.format(param, value))
        return self.wait_for('OK:SET:', timeout=3)

    def get_param(self, param):
        self.send('GET:{}'.format(param))
        line = self.wait_for('DATA:', timeout=3)
        if not line:
            return None
        parts = line.split(':')
        if len(parts) >= 3:
            return parts[2]
        return None

    def get_distances(self):
        self.send('DIST')
        start = time.time()
        while time.time() - start < 3:
            line = self.readline(timeout=0.2)
            if line and line.startswith('DATA:FL:'):
                parts = line.split(':')
                try:
                    return {
                        'fl': int(parts[2]),
                        'fr': int(parts[4]),
                        'sl': int(parts[6]),
                        'sr': int(parts[8]),
                    }
                except (ValueError, IndexError):
                    pass
        return {'fl': 999, 'fr': 999, 'sl': 999, 'sr': 999}

    # ================================================================
    # PID Control
    # ================================================================
    def align_wall(self):
        """PID wall alignment -> WALIGN"""
        return self.send_and_wait('WALIGN', 'DONE:WALIGN', timeout=50) is not None

    def front_to_distance(self, target_cm=25.0):
        """PID front distance control -> FRONTPID:target"""
        cmd = 'FRONTPID:{:.1f}'.format(float(target_cm))
        return self.send_and_wait(cmd, 'DONE:FRONTPID', timeout=50) is not None

    # ================================================================
    # Photo Sync (WAIT_SHOT / CONTINUE protocol)
    # ================================================================
    def _move_with_photo_sync(self, cmd, done_prefix, on_photo_ready=None, timeout=300):
        """Handle SHOTL/SHOTR WAIT_SHOT/CONTINUE protocol."""
        self.send(cmd)
        start = time.time()
        ok = True

        while time.time() - start < timeout:
            line = self.readline(timeout=0.2)
            if line is None or line == '':
                continue

            if line.startswith('WAIT_SHOT:'):
                parts = line.split(':')
                direction = parts[1] if len(parts) >= 2 else 'UNKNOWN'
                step_info = parts[2] if len(parts) >= 3 else ''
                print('[SYNC] Stopped and aligned: {} {}'.format(direction, step_info))

                try:
                    if on_photo_ready:
                        ok = bool(on_photo_ready(direction, step_info)) and ok
                except TypeError:
                    if on_photo_ready:
                        ok = bool(on_photo_ready(direction)) and ok
                except Exception as e:
                    ok = False
                    print('[ERR] Photo callback error: {}'.format(e))

                self.send('CONTINUE')

            elif line.startswith(done_prefix):
                return ok

            elif line.startswith('LOG:'):
                print('[Arduino] {}'.format(line[4:]))

            elif line.startswith('ERR:'):
                print('[Arduino ERROR] {}'.format(line[4:]))
                return False

        print('[ERR] Photo sync timeout')
        return False

    def strafe_left_photo(self, pwm=100, ms=1000, on_photo_ready=None):
        """Photo-synced strafe left -> SHOTL:pwm:ms"""
        return self._move_with_photo_sync(
            'SHOTL:{}:{}'.format(int(pwm), int(ms)),
            'DONE:SHOTL',
            on_photo_ready=on_photo_ready,
        )

    def strafe_right_photo(self, pwm=100, ms=1000, on_photo_ready=None):
        """Photo-synced strafe right -> SHOTR:pwm:ms"""
        return self._move_with_photo_sync(
            'SHOTR:{}:{}'.format(int(pwm), int(ms)),
            'DONE:SHOTR',
            on_photo_ready=on_photo_ready,
        )

    # ================================================================
    # Sequence Execution (WAIT:CAM + DONE:SEQ protocol)
    # ================================================================
    def run_sequence(self, trash_type, on_wait=None):
        """
        Execute sorting sequence -> SEQ:type

        trash_type: 'metal' / 'plastic' / 'auto'
        on_wait: callback(event_type, info) -> bool
                 Triggered when firmware sends WAIT:xxx.
                 Return True to continue, False to abort.

                 Events:
                   'CAM'  -> camera/recognition phase
                     manual_test: place trash manually, press Enter
                     auto_sort:  AI detection -> centering -> classify
                   'STEP' -> step-by-step advance (legacy STEP:N/M)
                     info='3/6' (step/total)
        """
        self.send('SEQ:{}'.format(trash_type.upper()))
        start = time.time()
        timeout = 300  # 5 min total timeout

        while time.time() - start < timeout:
            line = self.readline(timeout=0.2)
            if line is None or line == '':
                continue

            # WAIT:CAM / WAIT:xxx
            if line.startswith('WAIT:'):
                parts = line.split(':', 1)
                event_type = parts[1] if len(parts) >= 2 else 'UNKNOWN'
                info = parts[2] if len(parts) >= 3 else ''

                print('\n[WAIT:{}] Arduino paused, waiting for Pi...'.format(event_type))

                proceed = True
                if on_wait:
                    proceed = bool(on_wait(event_type, info))

                if proceed:
                    self.send('CONTINUE')
                else:
                    print('[STOP] Sequence aborted')
                    self.send('STOP')
                    return False

            # Legacy STEP:N/M
            elif line.startswith('STEP:'):
                step_part = line.split(':', 1)[1] if ':' in line else ''
                nums = step_part.split('/')
                step_num = int(nums[0]) if nums else 0
                total_steps = int(nums[1]) if len(nums) >= 2 else 0

                print('\n[Step {}/{}] Waiting for confirmation...'.format(step_num, total_steps))

                proceed = True
                if on_wait:
                    proceed = bool(on_wait('STEP', '{}/{}'.format(step_num, total_steps)))

                if proceed:
                    self.send('CONTINUE')
                else:
                    print('[STOP] Sequence aborted')
                    self.send('STOP')
                    return False

            # Done
            elif line.startswith('DONE:SEQ'):
                return 'ABORTED' not in line

            elif line.startswith('LOG:'):
                print('[Arduino] {}'.format(line[4:]))

            elif line.startswith('ERR:'):
                print('[Arduino ERROR] {}'.format(line[4:]))
                return False

        print('[ERR] Sequence timeout')
        return False


def find_arduino_port():
    """Auto-detect Arduino serial port."""
    for pattern in ['/dev/ttyUSB*', '/dev/ttyACM*', '/dev/ttyAMA*']:
        ports = glob.glob(pattern)
        if ports:
            return ports[0]
    return None


if __name__ == '__main__':
    port = find_arduino_port()
    if not port:
        print('Arduino not found')
        sys.exit(1)

    bot = RobotSerial(port)
    bot.set_debug(True)

    if bot.connect():
        print(bot.get_distances())
        bot.disconnect()
