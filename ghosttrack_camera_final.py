#!/usr/bin/env python3
# =============================================================================
# GHOSTTRACK — RASPBERRY PI 4 CAMERA LOOP CODE
# Final Version — mmWave Radar + Servo Motors + MicroSD
# =============================================================================
# Team    : Sumith | Rajamaran | Leharin
# Version : Final | May 2025
#
# COMPONENTS THIS CODE WORKS WITH:
#   - 160 degree USB Camera     → captures live video
#   - Raspberry Pi 4            → runs YOLOv8 dual model inference
#   - ESP32 (via UART)          → receives ALERT signal
#   - 24 GHz mmWave Radar       → distance sensing (ESP32 reads this)
#   - PCA9685 + Servo Motors x2 → directional alert (ESP32 controls this)
#   - MicroSD Module            → Ghost Log CSV (ESP32 writes this)
#   - LED Yellow + Red          → visual alerts (ESP32 controls this)
#
# WHAT THIS CODE DOES ON PI 4:
#   1. Opens 160 degree USB camera
#   2. Runs YOLOv8n COCO model  → detects person, bicycle, motorcycle
#   3. Runs GhostTrack model    → detects auto_rickshaw, e_rickshaw, tractor
#   4. Determines LEFT or RIGHT danger side from bounding box position
#   5. Estimates distance from bounding box size
#   6. Sends ALERT:LEFT:CRITICAL or SAFE to ESP32 via UART
#   7. Saves every event to Ghost Log CSV on Pi 4 as backup
#   8. Shows live annotated video on screen
#
# HOW TO RUN ON RASPBERRY PI 4:
#   python3 ghosttrack_camera_final.py
#
# INSTALL ON PI 4 FIRST:
#   pip install ultralytics opencv-python pyserial
# =============================================================================

import cv2
import csv
import datetime
import time
import os
import serial
from ultralytics import YOLO

# =============================================================================
# CONFIGURATION
# =============================================================================

# Model paths — copy these files to Pi 4 from Google Drive
MODEL_COCO_PATH  = 'yolov8n.pt'                         # auto downloads
MODEL_GHOST_PATH = '/home/pi/ghosttrack_best.onnx'      # copy from Drive

# Camera settings
CAMERA_INDEX  = 0        # 0 = first USB camera
FRAME_WIDTH   = 640
FRAME_HEIGHT  = 480
SHOW_DISPLAY  = True     # False if running without monitor

# ESP32 UART settings
ESP32_PORT    = '/dev/ttyUSB0'   # run "ls /dev/ttyUSB*" to find port
ESP32_BAUD    = 115200

# Detection confidence
CONF_THRESHOLD = 0.30

# Ghost Log on Pi 4 (backup — ESP32 also writes to MicroSD)
GHOST_LOG_PATH = '/home/pi/ghost_log_pi4.csv'

# Speed adaptive threshold
# Set to True if truck is on highway (>40 kmph)
HIGHWAY_MODE  = False

# =============================================================================
# CLASS DEFINITIONS
# =============================================================================

# COCO pretrained — detects person, bicycle, motorcycle well
COCO_CLASSES = {
    0: 'person',
    1: 'bicycle',
    3: 'motorcycle',
    2: 'car',
    5: 'bus',
    7: 'truck'
}

# GhostTrack trained model — detects Indian specific classes
GHOST_CLASSES = {
    0: 'person',
    1: 'bicycle',
    2: 'motorcycle',
    3: 'auto_rickshaw',
    4: 'e_rickshaw',
    5: 'car',
    6: 'lcv',
    7: 'bus',
    8: 'truck',
    9: 'tractor'
}

# Only these classes trigger ALERT to ESP32
VRU_CLASSES = {'person', 'bicycle', 'motorcycle', 'auto_rickshaw', 'e_rickshaw'}

# Box colors for display
COLORS = {
    'person':       (0,   0,  255),   # red
    'bicycle':      (0, 165,  255),   # orange
    'motorcycle':   (0,   0,  200),   # dark red
    'auto_rickshaw':(255, 0,  150),   # purple
    'e_rickshaw':   (200, 0,  200),   # magenta
    'car':          (0,  255,   0),   # green
    'bus':          (255, 200,  0),   # cyan
    'truck':        (100, 100, 100),  # grey
    'tractor':      (0,  100, 200),   # blue
    'lcv':          (0,  200, 100),   # teal
}

# =============================================================================
# LOAD MODELS
# =============================================================================

print("=" * 55)
print("  GHOSTTRACK Pi 4 Camera Loop — Final Version")
print("  mmWave Radar + Servo + MicroSD")
print("=" * 55)

print("\nLoading YOLOv8n COCO model (person/bicycle/motorcycle)...")
model_coco = YOLO(MODEL_COCO_PATH)
print("✅ COCO model ready")

print("Loading GhostTrack model (auto_rickshaw/tractor/Indian)...")
model_ghost = YOLO(MODEL_GHOST_PATH)
print("✅ GhostTrack model ready")

# =============================================================================
# CONNECT TO ESP32 VIA UART
# =============================================================================

esp32 = None
try:
    esp32 = serial.Serial(ESP32_PORT, ESP32_BAUD, timeout=1)
    time.sleep(2)  # wait for ESP32 to boot
    print(f"✅ ESP32 connected on {ESP32_PORT}")
except Exception as e:
    print(f"⚠️  ESP32 not connected ({e})")
    print("   Simulation mode — signals printed to terminal")

# =============================================================================
# OPEN CAMERA
# =============================================================================

print(f"\nOpening 160 degree USB camera...")
cap = cv2.VideoCapture(CAMERA_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    print("❌ Camera not found! Check USB connection.")
    print("   Try: CAMERA_INDEX = 1 or 2")
    exit(1)

actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"✅ Camera opened at {actual_w}x{actual_h}")

# =============================================================================
# GHOST LOG SETUP (Pi 4 backup log)
# =============================================================================

ghost_ready = False

def write_ghost_log(timestamp, cls, dist, zone, side, conf, source):
    global ghost_ready
    if not ghost_ready:
        with open(GHOST_LOG_PATH, 'w', newline='') as f:
            csv.writer(f).writerow([
                'timestamp', 'class', 'distance_m',
                'zone', 'side', 'confidence', 'alert_type', 'source'
            ])
        ghost_ready = True
    alert_type = 'CRITICAL' if zone == 'CRITICAL' else 'AWARENESS'
    with open(GHOST_LOG_PATH, 'a', newline='') as f:
        csv.writer(f).writerow([
            timestamp, cls, f'{dist:.1f}',
            zone, side, f'{conf:.2f}', alert_type, source
        ])

# =============================================================================
# HELPER FUNCTIONS
# =============================================================================

def estimate_distance(box_h_ratio):
    """
    Estimate distance from bounding box height.
    mmWave radar on ESP32 gives precise distance.
    This is a camera-based estimate as secondary reference.
    Speed-adaptive: highway mode expands thresholds.
    """
    if HIGHWAY_MODE:
        # Highway thresholds — larger safety margin
        if   box_h_ratio > 0.30: return 'CRITICAL', 2.0
        elif box_h_ratio > 0.15: return 'NEAR',     5.0
        else:                    return 'FAR',       9.0
    else:
        # City thresholds
        if   box_h_ratio > 0.40: return 'CRITICAL', 1.5
        elif box_h_ratio > 0.20: return 'NEAR',     3.5
        else:                    return 'FAR',       7.0

def get_danger_side(x_center_ratio):
    """Left half of frame = LEFT blind spot. Right half = RIGHT."""
    return 'LEFT' if x_center_ratio < 0.5 else 'RIGHT'

def send_alert(side, zone):
    """Send ALERT:SIDE:ZONE to ESP32."""
    signal = f'ALERT:{side}:{zone}\n'
    if esp32 and esp32.is_open:
        try:
            esp32.write(signal.encode())
        except Exception as e:
            print(f"⚠️  UART write failed: {e}")
    else:
        print(f'  📡 [SIM] ESP32 → {signal.strip()}')

def send_safe():
    """Send SAFE to ESP32 when no VRU detected."""
    if esp32 and esp32.is_open:
        try:
            esp32.write(b'SAFE\n')
        except:
            pass

def draw_hud(frame, fps, alert_count, vru_found, worst_zone):
    """Draw HUD overlay on frame."""
    h, w = frame.shape[:2]

    # Status bar at top
    bar_color = (0, 0, 200) if worst_zone == 'CRITICAL' else \
                (0, 165, 0) if vru_found else (30, 30, 30)
    cv2.rectangle(frame, (0, 0), (w, 52), bar_color, -1)

    status_text = "CRITICAL ALERT" if worst_zone == 'CRITICAL' else \
                  "VRU DETECTED" if vru_found else "SAFE"
    cv2.putText(frame, f"GHOSTTRACK  |  {status_text}",
                (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    cv2.putText(frame, f"FPS: {fps:.1f}  |  Alerts: {alert_count}  |  {'HIGHWAY' if HIGHWAY_MODE else 'CITY'} MODE",
                (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

    # mmWave indicator
    cv2.putText(frame, "mmWave: ESP32",
                (w - 140, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 200), 1)
    cv2.putText(frame, "Servo: ESP32",
                (w - 140, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 200), 1)

    return frame

# =============================================================================
# MAIN CAMERA LOOP
# =============================================================================

print(f"\n🚀 GhostTrack ACTIVE — monitoring blind spots")
print(f"   Mode: {'HIGHWAY' if HIGHWAY_MODE else 'CITY'}")
print(f"   Press Q to quit | Press H to toggle highway mode\n")

frame_count    = 0
alert_count    = 0
last_safe_sent = 0
fps_start      = time.time()
last_alert_time = 0
ALERT_COOLDOWN  = 0.5  # seconds between alerts to avoid flooding ESP32

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("⚠️  Frame read failed — retrying...")
            time.sleep(0.05)
            continue

        frame_count += 1
        h, w        = frame.shape[:2]
        all_dets    = []
        vru_found   = False
        worst_zone  = 'FAR'
        worst_side  = 'NONE'

        # ── MODEL 1: COCO (person, bicycle, motorcycle, car, bus, truck) ──
        r1 = model_coco(frame, conf=CONF_THRESHOLD, verbose=False)[0]
        for box in r1.boxes:
            cid = int(box.cls[0])
            if cid in COCO_CLASSES:
                all_dets.append({
                    'class':  COCO_CLASSES[cid],
                    'conf':   float(box.conf[0]),
                    'box':    box.xyxy[0].tolist(),
                    'source': 'COCO'
                })

        # ── MODEL 2: GHOST (auto_rickshaw, e_rickshaw, tractor, lcv) ────
        r2 = model_ghost(frame, conf=CONF_THRESHOLD, verbose=False)[0]
        for box in r2.boxes:
            cid   = int(box.cls[0])
            cname = GHOST_CLASSES.get(cid, 'unknown')
            if cname in ['auto_rickshaw', 'e_rickshaw', 'tractor', 'lcv']:
                all_dets.append({
                    'class':  cname,
                    'conf':   float(box.conf[0]),
                    'box':    box.xyxy[0].tolist(),
                    'source': 'GHOST'
                })

        # ── PROCESS DETECTIONS ───────────────────────────────────────────
        ts = datetime.datetime.now().isoformat()

        for d in all_dets:
            x1, y1, x2, y2 = d['box']
            bh    = (y2 - y1) / h
            xc    = ((x1 + x2) / 2) / w
            zone, dist = estimate_distance(bh)
            side       = get_danger_side(xc)
            cname      = d['class']

            # Write to Pi 4 Ghost Log
            write_ghost_log(ts, cname, dist, zone, side, d['conf'], d['source'])

            # VRU logic
            if cname in VRU_CLASSES:
                vru_found = True
                alert_count += 1

                # Track worst case for this frame
                if zone == 'CRITICAL':
                    worst_zone = 'CRITICAL'
                    worst_side = side
                elif zone == 'NEAR' and worst_zone != 'CRITICAL':
                    worst_zone = 'NEAR'
                    worst_side = side

                # Terminal output
                print(f"  ⚠️  {cname.upper()} | {side} | "
                      f"{dist:.1f}m ({zone}) | conf:{d['conf']:.2f} | [{d['source']}]")

            # Draw bounding box on frame
            color = COLORS.get(cname, (200, 200, 200))
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            label = f"{cname} {d['conf']:.2f}"
            cv2.putText(frame, label,
                        (int(x1), max(int(y1) - 6, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

            # Draw danger side arrow for VRUs
            if cname in VRU_CLASSES and zone in ('CRITICAL', 'NEAR'):
                arrow_x = 20 if side == 'LEFT' else w - 20
                cv2.arrowedLine(frame,
                    (w // 2, h - 30),
                    (arrow_x, h - 30),
                    (0, 0, 255), 3, tipLength=0.3)

        # ── SEND UART SIGNAL TO ESP32 ────────────────────────────────────
        now = time.time()
        if vru_found and worst_zone in ('CRITICAL', 'NEAR'):
            if now - last_alert_time >= ALERT_COOLDOWN:
                send_alert(worst_side, worst_zone)
                last_alert_time = now
        else:
            if now - last_safe_sent > 2.0:
                send_safe()
                last_safe_sent = now

        # ── HUD OVERLAY ──────────────────────────────────────────────────
        fps = frame_count / (time.time() - fps_start)
        frame = draw_hud(frame, fps, alert_count, vru_found, worst_zone)

        # ── DISPLAY ──────────────────────────────────────────────────────
        if SHOW_DISPLAY:
            cv2.imshow('GhostTrack — Blind Spot Monitor', frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("\nQ pressed — stopping...")
                break
            elif key == ord('h'):
                HIGHWAY_MODE = not HIGHWAY_MODE
                print(f"Switched to {'HIGHWAY' if HIGHWAY_MODE else 'CITY'} mode")

except KeyboardInterrupt:
    print("\n\nStopping GhostTrack...")

finally:
    # Cleanup
    send_safe()
    cap.release()
    if SHOW_DISPLAY:
        cv2.destroyAllWindows()
    if esp32 and esp32.is_open:
        esp32.close()

    print("\n" + "=" * 40)
    print("  GHOSTTRACK STOPPED")
    print("=" * 40)
    print(f"  Total frames : {frame_count}")
    print(f"  Total alerts : {alert_count}")
    print(f"  Ghost Log    : {GHOST_LOG_PATH}")
    print("=" * 40)

