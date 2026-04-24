#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AI Auto Trash Sorting (MobileNetV2 + Background Subtraction)
=============================================================
Full pipeline:
  1. Pi sends SEQ:AUTO -> firmware does FRONTPID:25 + WALIGN -> pause
  2. Firmware sends WAIT:CAM -> Pi runs AI recognition:
     a. Pi Camera captures frame
     b. Background subtraction -> find trash bbox (x, y, w, h)
     c. MobileNetV2 classification -> metal / plastic
     d. Compute offset from image center
     e. If offset too large -> send SL/SR -> firmware WALIGN -> re-capture -> loop
     f. Once centered -> send TYPE:<class> -> CONTINUE
  3. Firmware continues: RELEASE -> FRONTPID:6 -> GRAB -> Phase 2 -> Phase 3

Usage: python auto_sort.py
Prerequisites:
  - Firmware flashed (firmware/trash_robot_firmware.ino)
  - robot_serial.py in the same directory
  - classifier_mobile.pt + bg_empty.jpg in src/model/
  - Pi Camera (CSI, libcamera stack) connected
  - espeak-ng installed (for voice feedback)

Dependencies: pip install opencv-python torch torchvision picamera2
"""

import os
import sys
import time
import json
import subprocess
import cv2
import numpy as np
import torch
import torch.nn as nn
from torchvision import transforms, models
from PIL import Image

sys.path.insert(0, os.path.dirname(__file__))
from robot_serial import RobotSerial, find_arduino_port

# ======================== Config ========================
CLASSES = ['metal', 'plastic']
IMG_SIZE = 224
MIN_OBJ_AREA = 500
CONF_THRESH = 0.5

# Centering thresholds (pixels)
# Target: object center should be at img_cx + TARGET_OFFSET_PX
TARGET_OFFSET_PX = 30
CENTER_TOLERANCE_PX = 15
MAX_CENTERING_ATTEMPTS = 10

# Strafe estimation (pixel offset -> timed strafe in ms)
PIXEL_PER_CM = 30
TIME_STRAFE_PER_CM = 40
STRAFE_DAMPING = 0.3           # damping factor (0~1), lower = more conservative
PWM_STRAFE_VALUE = 100

# Loop control
MAX_CYCLES = 0                 # max cycles, 0 = unlimited (Ctrl+C to stop)

# Voice
VOICE_ENABLED = True
VOICE_SPEED = 150

# Pi Camera (CSI, libcamera)
CAM_WIDTH = 640
CAM_HEIGHT = 480

# Background subtraction
BG_DIFF_THRESH = 25
BG_DIFF_BLUR = 21

# ======================== File Paths ========================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(SCRIPT_DIR, 'model', 'classifier_mobile.pt')
BG_PATH = os.path.join(SCRIPT_DIR, 'model', 'bg_empty.jpg')
CALIB_FILE = os.path.join(SCRIPT_DIR, 'config', 'calibration.json')
GRIP_FILE = os.path.join(SCRIPT_DIR, 'config', 'grip_angles.json')


# ======================== Classifier ========================
class TrashClassifier(nn.Module):
    def __init__(self, num_classes):
        super().__init__()
        self.model = models.mobilenet_v2(pretrained=False)
        in_features = self.model.classifier[1].in_features
        self.model.classifier[1] = nn.Sequential(
            nn.Dropout(p=0.3),
            nn.Linear(in_features, 256),
            nn.ReLU(inplace=True),
            nn.Dropout(p=0.2),
            nn.Linear(256, num_classes)
        )

    def forward(self, x):
        return self.model(x)


def load_classifier(model_path):
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print('[INIT] Device: {}'.format(device))
    ckpt = torch.load(model_path, map_location=device, weights_only=False)
    num_classes = ckpt.get('num_classes', len(CLASSES))
    model = TrashClassifier(num_classes)
    sd = ckpt['model_state_dict']
    first_key = list(sd.keys())[0]
    if first_key.startswith('model.'):
        model.load_state_dict(sd)
    else:
        model.load_state_dict({f'model.{k}': v for k, v in sd.items()})
    model.to(device)
    model.eval()
    classes = ckpt.get('classes', CLASSES)
    print('[INIT] Model loaded: {} classes={}'.format(model_path, classes))
    return model, classes, device


val_transform = transforms.Compose([
    transforms.Resize((IMG_SIZE, IMG_SIZE)),
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
])


def predict_crop(model, crop_rgb, classes, device):
    """Classify a cropped RGB region."""
    pil_img = Image.fromarray(crop_rgb)
    tensor = val_transform(pil_img).unsqueeze(0).to(device)
    with torch.no_grad():
        output = model(tensor)
        probs = torch.softmax(output, dim=1)
        conf, pred = probs.max(1)
    class_id = pred.item()
    confidence = conf.item()
    class_name = classes[class_id]
    return class_name, confidence, class_id


# ======================== Background Subtraction ========================
class BgSubSegmenter:
    def __init__(self, bg_image_path):
        bg = cv2.imread(bg_image_path)
        if bg is None:
            raise FileNotFoundError('Cannot read background: {}'.format(bg_image_path))
        self.bg = cv2.GaussianBlur(bg, (BG_DIFF_BLUR, BG_DIFF_BLUR), 0)
        print('[INIT] Background loaded: {} ({}x{})'.format(
            bg_image_path, bg.shape[1], bg.shape[0]))

    def segment(self, frame_bgr, min_area=MIN_OBJ_AREA):
        """Background subtraction segmentation. Returns [(x, y, w, h), ...]"""
        bg_h, bg_w = self.bg.shape[:2]
        if frame_bgr.shape[:2] != (bg_h, bg_w):
            frame_bgr = cv2.resize(frame_bgr, (bg_w, bg_h))

        blurred = cv2.GaussianBlur(frame_bgr, (BG_DIFF_BLUR, BG_DIFF_BLUR), 0)
        bg_gray = cv2.cvtColor(self.bg, cv2.COLOR_BGR2GRAY)
        frame_gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)

        diff = cv2.absdiff(bg_gray, frame_gray)
        _, fg_mask = cv2.threshold(diff, BG_DIFF_THRESH, 255, cv2.THRESH_BINARY)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, kernel, iterations=3)
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, kernel, iterations=2)

        contours_fill, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(fg_mask, contours_fill, -1, 255, -1)

        contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        img_h, img_w = frame_bgr.shape[:2]
        results = []

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if w > img_w * 0.95 and h > img_h * 0.95:
                continue
            results.append((x, y, w, h))

        return results, frame_bgr


# ======================== Voice ========================
def speak(text, lang='en'):
    """Speak text using espeak-ng (offline, pre-installed on RPi)."""
    if not VOICE_ENABLED:
        return
    try:
        subprocess.run(
            ['espeak-ng', '-v', lang, '-s', str(VOICE_SPEED), text],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except FileNotFoundError:
        try:
            subprocess.run(
                ['espeak', '-v', lang, '-s', str(VOICE_SPEED), text],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except FileNotFoundError:
            print('[WARN] espeak not installed, cannot speak: {}'.format(text))


def speak_class(class_name):
    """Speak the detected trash class."""
    speak(class_name)


# ======================== Camera ========================
class Camera:
    def __init__(self):
        try:
            from picamera2 import Picamera2
        except ImportError:
            raise RuntimeError(
                'picamera2 not installed. Run: sudo apt install python3-picamera2'
            )

        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"size": (CAM_WIDTH, CAM_HEIGHT),
                  "format": "RGB888"},
        )
        self.picam2.configure(config)
        self.picam2.start()
        # Pi Camera needs time to stabilize exposure and white balance
        time.sleep(2)
        frame = self.picam2.capture_array()
        h, w = frame.shape[:2]
        print('[INIT] Pi Camera ready (picamera2 + libcamera): {}x{}'.format(w, h))

    def capture(self):
        """Capture one frame, returns RGB numpy array."""
        frame = self.picam2.capture_array()
        if frame is None:
            return None
        return frame

    def release(self):
        self.picam2.stop()


# ======================== AI Detection & Centering ========================

# Flag to avoid repeating voice announcements during centering
_speech_class_name = [None]

def detect_and_classify(camera, segmenter, model, classes, device):
    """
    Capture -> segment -> classify.
    Returns: (class_name, confidence, cx, cy, bbox) or None
    """
    frame = camera.capture()
    if frame is None:
        print('[AI] Failed to capture image')
        return None

    rois, _ = segmenter.segment(frame)
    if not rois:
        print('[AI] No object detected')
        return None

    # Pick the largest region
    rois.sort(key=lambda r: r[2] * r[3], reverse=True)
    x, y, w, h = rois[0]

    crop = frame[y:y+h, x:x+w].copy()
    if crop.shape[0] < 20 or crop.shape[1] < 20:
        print('[AI] Object too small')
        return None

    class_name, confidence, class_id = predict_crop(model, crop, classes, device)

    if confidence < CONF_THRESH:
        print('[AI] Low confidence: {} ({:.2f})'.format(class_name, confidence))
        return None

    cx = x + w // 2
    cy = y + h // 2
    img_cx = frame.shape[1] // 2
    img_cy = frame.shape[0] // 2

    print('[AI] Detected: {} ({:.1%}) bbox=({},{},{},{}) center=({},{}) image_center=({},{})'.format(
        class_name, confidence, x, y, w, h, cx, cy, img_cx, img_cy))

    return class_name, confidence, cx, cy, (x, y, w, h)


def center_and_classify(bot, camera, segmenter, model, classes, device):
    """
    Full detection + centering pipeline:
    1. Capture and detect
    2. Compute offset
    3. Strafe + wall-align
    4. Loop until centered
    5. Return (class_name, confidence)

    Returns: (class_name, confidence) or None
    """
    for attempt in range(1, MAX_CENTERING_ATTEMPTS + 1):
        print('\n[AI] === Centering attempt {}/{} ==='.format(attempt, MAX_CENTERING_ATTEMPTS))

        result = detect_and_classify(camera, segmenter, model, classes, device)
        if result is None:
            print('[AI] No object detected, aborting')
            return None

        class_name, confidence, cx, cy, bbox = result
        frame = camera.capture()
        if frame is not None:
            img_cx = frame.shape[1] // 2
        else:
            img_cx = 320

        # Compute horizontal offset (pixels): target = img_cx + TARGET_OFFSET_PX
        target_x = img_cx + TARGET_OFFSET_PX
        offset_px = cx - target_x
        print('[AI] Object cx={}, target={}, offset={}px (tolerance: {}px)'.format(
            cx, target_x, offset_px, CENTER_TOLERANCE_PX))

        if abs(offset_px) <= CENTER_TOLERANCE_PX:
            print('[AI] Object is CENTERED! Type: {} ({:.1%})'.format(
                class_name, confidence))
            return class_name, confidence

        # Compute strafe distance (cm) and direction, apply damping
        offset_cm = offset_px / PIXEL_PER_CM
        strafe_ms = int(abs(offset_cm) * TIME_STRAFE_PER_CM * STRAFE_DAMPING)

        if strafe_ms < 20:
            print('[AI] Offset too small ({}ms), considered centered'.format(strafe_ms))
            return class_name, confidence

        # Cap single strafe at 200ms
        strafe_ms = min(strafe_ms, 200)

        if offset_px > 0:
            # Object is right of target -> robot needs to move right
            direction = 'RIGHT'
            bot._send('SR:{}:{}'.format(PWM_STRAFE_VALUE, strafe_ms))
        else:
            # Object is left of target -> robot needs to move left
            direction = 'LEFT'
            bot._send('SL:{}:{}'.format(PWM_STRAFE_VALUE, strafe_ms))

        print('[AI] Strafe {} for {}ms ({}cm, damping={})'.format(
            direction, strafe_ms, abs(offset_cm), STRAFE_DAMPING))

        # Wait for Arduino to finish
        time.sleep(strafe_ms / 1000.0 + 0.5)

        # Re-align to wall after each strafe
        print('[AI] Re-aligning wall...')
        bot._send('WALIGN')
        time.sleep(5)

        # Clear serial buffer to avoid interfering with WAIT:CAM protocol
        bot.ser.reset_input_buffer()

    print('[AI] Max centering attempts reached')
    return None


# ======================== WAIT Callback ========================
def make_on_wait_ai(bot, camera, segmenter, model, classes, device):
    """
    Create AI WAIT callback for auto_sort.

    On WAIT:CAM:
    0. Reset voice flag
    1. AI detection + centering
    2. Speak the class name
    3. Send TYPE:<class> to firmware
    4. Return True (CONTINUE)
    """
    def on_wait_ai(event_type, info):
        # Reset voice flag for each new recognition
        _speech_class_name[0] = None
        if event_type == 'CAM':
            print('')
            print('  ========================================')
            print('  [WAIT:CAM] AI Recognition Phase')
            print('  ========================================')

            result = center_and_classify(
                bot, camera, segmenter, model, classes, device)

            if result is None:
                print('[AI] Recognition failed, stopping sequence')
                return False

            class_name, confidence = result

            # Speak the result (only once per recognition, not during centering retries)
            if _speech_class_name[0] != class_name:
                _speech_class_name[0] = class_name
                speak_class(class_name)

            # Send type to firmware (used in Phase 3)
            print('[AI] Sending TYPE:{}'.format(class_name.upper()))
            bot._send('TYPE:{}'.format(class_name.upper()))
            time.sleep(0.1)

            # Brief pause for firmware to process TYPE command
            time.sleep(0.2)

            print('[AI] Sending CONTINUE')
            return True

        elif event_type == 'STEP':
            print('\n[Step {}] Auto-continue'.format(info))
            return True

        else:
            return True

    return on_wait_ai


# ======================== Param Loading ========================
def load_params():
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
    print('=== Uploading params to Arduino ===')
    for key, value in params.items():
        result = bot.set_param(key, value)
        if result:
            print('  {} = {}'.format(key, value))
    print('[OK] Params uploaded')


# ======================== Main ========================
def main():
    print('=' * 60)
    print('AI Auto Trash Sorting')
    print('Model: MobileNetV2 + Background Subtraction')
    print('=' * 60)

    # Load params
    params = load_params()
    print('\nLoaded params:')
    for k, v in params.items():
        print('  {} = {}'.format(k, v))

    # Load AI model
    print('\n--- Loading AI Model ---')
    model, classes, device = load_classifier(MODEL_PATH)

    # Load background subtractor
    print('\n--- Loading Background Subtractor ---')
    segmenter = BgSubSegmenter(BG_PATH)

    # Open camera
    print('\n--- Opening Camera ---')
    camera = Camera()

    # Connect to Arduino
    print('\n--- Connecting to Arduino ---')
    port = find_arduino_port()
    if not port:
        print('[ERR] Arduino not found')
        camera.release()
        sys.exit(1)

    bot = RobotSerial(port)
    bot.set_debug(True)

    if not bot.connect():
        camera.release()
        sys.exit(1)

    # Upload params
    upload_all_params(bot, params)

    # Create AI callback
    on_wait = make_on_wait_ai(bot, camera, segmenter, model, classes, device)

    try:
        while True:
            print('\n' + '=' * 60)
            print('Menu:')
            print('  1. Start auto sort loop (SEQ:AUTO)')
            print('  2. Single auto sort (one cycle)')
            print('  --- Tests ---')
            print('  3. Test camera capture')
            print('  4. Test AI detection (no robot movement)')
            print('  5. Read distances')
            print('  6. Manual command')
            print('  7. Re-upload params')
            print('  8. Test voice')
            print('  0. Exit')
            print('=' * 60)

            choice = input('Select: ').strip()

            if choice == '0':
                break

            elif choice == '1':
                max_count = MAX_CYCLES
                print('\n[LOOP] Starting auto sort loop')
                if max_count > 0:
                    print('[LOOP] Max cycles: {}'.format(max_count))
                else:
                    print('[LOOP] Unlimited mode (Ctrl+C to stop)')
                count = 0
                try:
                    while True:
                        if max_count > 0 and count >= max_count:
                            print('\n[LOOP] Completed {} cycles, stopping'.format(count))
                            speak('All done')
                            break

                        count += 1
                        print('\n\n' + '#' * 50)
                        print('# Cycle {}'.format(count))
                        print('#' * 50)

                        # Close gripper before next cycle (not the first)
                        if count > 1:
                            print('[PREP] Closing gripper for next grab')
                            bot.grab()
                            time.sleep(0.3)

                        success = bot.run_sequence('auto', on_wait=on_wait)
                        if success:
                            print('[OK] Cycle {} completed!'.format(count))
                        else:
                            print('[FAIL] Cycle {} failed'.format(count))
                            if max_count <= 0:
                                print('[LOOP] Retrying in 1s...')
                                time.sleep(1)
                            else:
                                break
                        time.sleep(0.5)
                except KeyboardInterrupt:
                    print('\n[LOOP] Stopped, {} cycles completed'.format(count))
                    speak('Stopped')
                    bot.stop()

            elif choice == '2':
                print('\n[SINGLE] Starting single auto sort...')
                success = bot.run_sequence('auto', on_wait=on_wait)
                if success:
                    print('[OK] Sort completed!')
                else:
                    print('[FAIL] Sort failed')

            elif choice == '3':
                print('\n[CAM] Testing camera...')
                frame = camera.capture()
                if frame is not None:
                    cv2.imshow('Camera Test', frame)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()
                    print('[OK] Image captured: {}x{}'.format(
                        frame.shape[1], frame.shape[0]))
                else:
                    print('[ERR] Capture failed')

            elif choice == '4':
                print('\n[AI] Testing detection...')
                result = detect_and_classify(
                    camera, segmenter, model, classes, device)
                if result:
                    class_name, confidence, cx, cy, bbox = result
                    print('[OK] Result: {} ({:.1%}) center=({},{}) bbox={}'.format(
                        class_name, confidence, cx, cy, bbox))
                    frame = camera.capture()
                    if frame is not None:
                        x, y, w, h = bbox
                        color = (0, 255, 0) if class_name == 'metal' else (255, 100, 0)
                        cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)
                        cv2.circle(frame, (cx, cy), 5, color, -1)
                        img_cx = frame.shape[1] // 2
                        img_cy = frame.shape[0] // 2
                        cv2.line(frame, (img_cx, 0), (img_cx, frame.shape[0]), (255, 255, 0), 1)
                        cv2.line(frame, (0, img_cy), (frame.shape[1], img_cy), (255, 255, 0), 1)
                        cv2.putText(frame, '{} {:.1%}'.format(class_name, confidence),
                                    (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                        cv2.imshow('AI Detection', frame)
                        cv2.waitKey(0)
                        cv2.destroyAllWindows()
                else:
                    print('[FAIL] No object detected')

            elif choice == '5':
                d = bot.get_distances()
                print('FL={} FR={} SL={} SR={}'.format(
                    d['fl'], d['fr'], d['sl'], d['sr']))

            elif choice == '6':
                cmd = input('Command: ').strip()
                if cmd:
                    bot._send(cmd)
                    time.sleep(2)

            elif choice == '7':
                upload_all_params(bot, params)

            elif choice == '8':
                print('[VOICE] Testing voice...')
                speak('metal')
                time.sleep(1)
                speak('plastic')

    except KeyboardInterrupt:
        print('\n[Interrupted]')
        bot.stop()

    finally:
        camera.release()
        bot.disconnect()
        print('[OK] Disconnected')


if __name__ == '__main__':
    main()
