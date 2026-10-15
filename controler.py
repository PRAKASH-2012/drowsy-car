import cv2
import mediapipe as mp
import numpy as np
import serial
import serial.tools.list_ports
import time
import sys
import os
import ctypes
import urllib.request

# Auto-detect Arduino Nano COM Port
def find_arduino_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None
    for p in ports:
        desc = p.description.lower()
        if any(term in desc for term in ['arduino', 'ch340', 'usb-serial', 'ftdi', 'serial']):
            return p.device
    return ports[0].device

PORT = find_arduino_port()
BAUD_RATE = 9600

ser = None
SIM_MODE = False
if PORT is None:
    print("[WARN] No Arduino detected. Running in SIMULATION mode (no serial).")
    print("       Plug in USB cable to drive the real car.")
    SIM_MODE = True
else:
    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=0.1)
        time.sleep(2)  # Wait for Arduino bootloader auto-reset
        print(f"[OK] Connected to Arduino Nano on {PORT}")
    except Exception as e:
        print(f"[WARN] Could not open {PORT}: {e}")
        print("       TIP: Ensure the Arduino IDE Serial Monitor is closed.")
        print("       Running in SIMULATION mode (no serial).")
        ser = None
        SIM_MODE = True

# --- MediaPipe Face Mesh Setup (supports legacy Solutions API and new Tasks API) ---
USE_TASKS_API = False
face_mesh = None
landmarker = None
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "face_landmarker.task")
MODEL_URL = "https://storage.googleapis.com/mediapipe-models/face_landmarker/face_landmarker/float16/1/face_landmarker.task"

try:
    mp_face_mesh = mp.solutions.face_mesh
    face_mesh = mp_face_mesh.FaceMesh(
        max_num_faces=1,
        refine_landmarks=True,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    )
    print("[OK] MediaPipe FaceMesh (legacy Solutions API) initialised.")
except AttributeError:
    # mediapipe >= 0.10.20 removed mp.solutions -> use Tasks Vision API
    print("[INFO] mp.solutions not found, using MediaPipe Tasks FaceLandmarker API...")
    try:
        from mediapipe.tasks import python as mp_python
        from mediapipe.tasks.python import vision

        if not os.path.exists(MODEL_PATH):
            print(f"[INFO] Downloading face landmarker model (~2.7MB)...")
            urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)
            print("[OK] Model downloaded.")

        base_options = mp_python.BaseOptions(model_asset_path=MODEL_PATH)
        options = vision.FaceLandmarkerOptions(
            base_options=base_options,
            output_face_blendshapes=False,
            output_facial_transformation_matrixes=False,
            num_faces=1,
            running_mode=vision.RunningMode.VIDEO
        )
        landmarker = vision.FaceLandmarker.create_from_options(options)
        USE_TASKS_API = True
        print("[OK] MediaPipe FaceLandmarker (Tasks API) initialised.")
    except Exception as e:
        print(f"[ERROR] Could not initialise MediaPipe: {e}")
        print("TIP: Use the project venv (mediapipe 0.10.14): .\\.venv\\Scripts\\python.exe controler.py")
        sys.exit(1)

LEFT_EYE = [33, 160, 158, 133, 153, 144]
RIGHT_EYE = [362, 385, 387, 263, 373, 380]

def calculate_ear(landmarks, eye_indices, img_w, img_h):
    pts = [np.array([landmarks[i].x * img_w, landmarks[i].y * img_h]) for i in eye_indices]
    d_v1 = np.linalg.norm(pts[1] - pts[5])
    d_v2 = np.linalg.norm(pts[2] - pts[4])
    d_h = np.linalg.norm(pts[0] - pts[3])
    return (d_v1 + d_v2) / (2.0 * d_h) if d_h != 0 else 0.0

def calculate_ear_tasks(landmarks, eye_indices, img_w, img_h):
    pts = [np.array([landmarks[i].x * img_w, landmarks[i].y * img_h]) for i in eye_indices]
    d_v1 = np.linalg.norm(pts[1] - pts[5])
    d_v2 = np.linalg.norm(pts[2] - pts[4])
    d_h = np.linalg.norm(pts[0] - pts[3])
    return (d_v1 + d_v2) / (2.0 * d_h) if d_h != 0 else 0.0

EAR_THRESHOLD = 0.22         # Eyelid closure ratio
SLEEP_TIME_LIMIT = 1.5       # Seconds required to declare drowsiness (half speed + alarm)
SLEEP_STOP_LIMIT = 10.0      # Seconds of continuous sleep -> full stop
AWAKE_CONFIRM_TIME = 1.0     # Eyes-open seconds required to auto-clear the alarm

eye_closed_start = None
eye_open_start = None
is_drowsy = False
drowsy_start = None        # when the current drowsy episode began
full_stop_sent = False     # 10s+ escalation sent for this episode
current_status = "AWAKE"
radio_status = "SIM MODE - no radio" if SIM_MODE else "Waiting for command"

# --- Speed control (sent with every motion command as CMD:speed) ---
MIN_SPEED = 80           # motors stall below ~70 on L298N
MAX_SPEED = 255
SPEED_STEP = 15
DEFAULT_SPEED = 210
speed = DEFAULT_SPEED
last_motion = None       # last drive command (FWD/REV/LFT/RGT/SLFT/SRGT) or None

# --- Hold-to-drive key polling (car stops the moment keys are released) ---
try:
    _user32 = ctypes.windll.user32
except Exception:
    _user32 = None
HOLD_TO_DRIVE = _user32 is not None

KEY_TO_MOTION = {'W': 'FWD', 'S': 'REV', 'A': 'LFT', 'D': 'RGT', 'Z': 'SLFT', 'C': 'SRGT'}
DRIVE_KEYS = tuple(KEY_TO_MOTION)
MOTION_LABEL = {'FWD': 'DRIVING FORWARD', 'REV': 'DRIVING REVERSE',
                'LFT': 'SPIN LEFT', 'RGT': 'SPIN RIGHT',
                'SLFT': 'GENTLE LEFT', 'SRGT': 'GENTLE RIGHT'}
held_order = []          # held drive keys, most-recently-pressed last
suppress_keys = set()    # keys ignored until released (after explicit STOP)
last_heartbeat = 0.0
HEARTBEAT_INTERVAL = 0.3  # re-send active motion (matches receiver watchdog)

def raw_held_keys():
    """Physical state of drive keys, independent of window focus."""
    if _user32 is None:
        return set()
    return {ch for ch in DRIVE_KEYS if _user32.GetAsyncKeyState(ord(ch)) & 0x8000}

def poll_drive_keys():
    """Update hold order; return held drive keys (most-recently-pressed last)."""
    global held_order, suppress_keys
    held = raw_held_keys()
    suppress_keys &= held          # stop ignoring a key once it is released
    held -= suppress_keys
    held_order = [ch for ch in held_order if ch in held]
    for ch in held:
        if ch not in held_order:
            held_order.append(ch)
    return held_order

def transmit_command(cmd_string):
    if ser is None:
        print(f"[SIM] Would send '{cmd_string}'")
        return
    if not getattr(ser, "is_open", False):
        print(f"[WARN] Serial port is closed; dropped '{cmd_string}'")
        return
    try:
        ser.write((cmd_string + '\n').encode('utf-8'))
        ser.flush()
        print(f"[SERIAL] Sent '{cmd_string}' to transmitter on {PORT}")
    except (serial.SerialException, OSError, ValueError):
        print(f"[WARN] Could not send '{cmd_string}' because the serial port is unavailable.")


def read_transmitter_status():
    global radio_status
    if ser is None or not getattr(ser, "is_open", False):
        return
    try:
        while ser.in_waiting:
            message = ser.readline().decode('utf-8', errors='replace').strip()
            if not message:
                continue
            print(f"[TRANSMITTER] {message}")
            if message.startswith("RF ACK:"):
                radio_status = "ACK - " + message.replace("RF ACK:", "", 1).strip()
            elif message.startswith("RF NO ACK:"):
                radio_status = "NO ACK - " + message.replace("RF NO ACK:", "", 1).strip()
            elif message.startswith("RF Sent:"):
                # Older transmitter firmware (re-flash with trans/trans.ino for ACK feedback)
                radio_status = "SENT - " + message.replace("RF Sent:", "", 1).strip() + " (no ACK info)"
    except (serial.SerialException, OSError, ValueError):
        print("[WARN] Serial transmitter status read failed; port is closing or unavailable.")

def send_motion(cmd):
    """Send a drive command tagged with current speed, e.g. 'FWD:210'."""
    global last_motion
    last_motion = cmd
    transmit_command(f"{cmd}:{speed}")

def adjust_speed(delta):
    """Change speed and apply live if the car is currently driving."""
    global speed
    speed = max(MIN_SPEED, min(MAX_SPEED, speed + delta))
    print(f"[INFO] Speed set to {speed}")
    if last_motion is not None and not is_drowsy:
        transmit_command(f"{last_motion}:{speed}")

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    cap = cv2.VideoCapture(1)
if not cap.isOpened():
    print("[ERROR] Could not open webcam (tried index 0 and 1).")
    print("TIP: Check camera is plugged in and not used by another app.")
    if ser is not None:
        ser.close()
    sys.exit(1)
print("[OK] Webcam opened.")

print("\n--- CONTROLS (hold-to-drive: car stops when keys are released) ---")
print("  W / S     : Hold for Forward / Reverse")
print("  A / D     : Hold to Spin Left / Right")
print("  Z / C     : Hold for Gentle Left / Right")
print("  + / -     : Speed up / down (applies live)")
print("  Space / X : Stop")
print("  R         : Reset Emergency Alarm (auto-resets when eyes open)")
print("  Q         : Quit")
print("  NOTE: Sleep alarm arms only while driving (parked = no alarm)\n")
if SIM_MODE:
    print("*** SIMULATION MODE: serial disabled, vision + keys still work ***\n")

frame_count = 0
try:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("[ERROR] Webcam stopped returning frames.")
            break

        read_transmitter_status()
        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        current_ear = 0.0
        now = time.time()
        face_found = False

        if USE_TASKS_API:
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
            result = landmarker.detect_for_video(mp_image, int(now * 1000))
            if result.face_landmarks:
                face_found = True
                mesh = result.face_landmarks[0]
                left_ear = calculate_ear_tasks(mesh, LEFT_EYE, w, h)
                right_ear = calculate_ear_tasks(mesh, RIGHT_EYE, w, h)
                current_ear = (left_ear + right_ear) / 2.0
        else:
            results = face_mesh.process(rgb_frame)
            if results.multi_face_landmarks:
                face_found = True
                mesh = results.multi_face_landmarks[0].landmark
                left_ear = calculate_ear(mesh, LEFT_EYE, w, h)
                right_ear = calculate_ear(mesh, RIGHT_EYE, w, h)
                current_ear = (left_ear + right_ear) / 2.0

        if face_found:
            if current_ear < EAR_THRESHOLD:
                eye_open_start = None
                if eye_closed_start is None:
                    eye_closed_start = now
                duration = now - eye_closed_start

                if duration >= SLEEP_TIME_LIMIT and not is_drowsy:
                    if last_motion is not None:
                        # Car is moving -> alarm + slowdown
                        is_drowsy = True
                        last_motion = None
                        drowsy_start = now
                        full_stop_sent = False
                        current_status = "DROWSY! ALARM & HALF SPEED"
                        transmit_command("DROWSY")
                        print(f"[ALERT] Driver asleep for {duration:.2f}s! Sent 'DROWSY'.")
                    else:
                        # Parked -> eyes closed, no alarm
                        current_status = "PARKED - eyes closed, alarm off"
            else:
                eye_closed_start = None
                if not is_drowsy:
                    current_status = "AWAKE"
                else:
                    # Eyes open while alarm is latched: confirm driver is
                    # really awake, then auto-reset (beeper/LED off).
                    if eye_open_start is None:
                        eye_open_start = now
                    elif now - eye_open_start >= AWAKE_CONFIRM_TIME:
                        is_drowsy = False
                        eye_open_start = None
                        drowsy_start = None
                        full_stop_sent = False
                        current_status = "AWAKE (auto-reset)"
                        transmit_command("RESET")
                        print("[INFO] Eyes open - alarm auto-cleared, RESET sent.")
        else:
            eye_closed_start = None
            eye_open_start = None

        # Escalation: still asleep after 10s -> full stop (alarm keeps on)
        if is_drowsy and not full_stop_sent and drowsy_start is not None \
                and (now - drowsy_start) >= SLEEP_STOP_LIMIT:
            full_stop_sent = True
            current_status = "SLEEPING 10s+ - FULL STOP"
            transmit_command("SLEEP_STOP")
            print(f"[ALERT] Driver asleep for {now - drowsy_start:.1f}s! Sent 'SLEEP_STOP'.")

        # Capture keyboard controls inside video window
        key = cv2.waitKey(25) & 0xFF

        if key == ord('q'):
            last_motion = None
            transmit_command("STOP")
            break
        elif key == ord('r'):
            is_drowsy = False
            eye_open_start = None
            last_motion = None
            drowsy_start = None
            full_stop_sent = False
            current_status = "SYSTEM RESET - AWAKE"
            transmit_command("RESET")
            print("[INFO] Reset sent. Alarm & LED turned off.")
        elif key == ord('+') or key == ord('='):
            adjust_speed(SPEED_STEP)
            if not is_drowsy:
                current_status = f"SPEED {speed}"
        elif key == ord('-') or key == ord('_'):
            adjust_speed(-SPEED_STEP)
            if not is_drowsy:
                current_status = f"SPEED {speed}"
        elif key == ord('x') or key == 32:
            # Explicit stop: also ignore still-held keys until re-pressed
            if not is_drowsy:
                last_motion = None
                held_order.clear()
                suppress_keys.update(raw_held_keys())
                transmit_command("STOP")
                current_status = "STOPPED"
        elif not is_drowsy:
            if HOLD_TO_DRIVE:
                pass  # driving handled by hold polling below
            elif key == ord('w'):
                send_motion("FWD")
                current_status = f"DRIVING FORWARD @ {speed}"
            elif key == ord('s'):
                send_motion("REV")
                current_status = f"DRIVING REVERSE @ {speed}"
            elif key == ord('a'):
                send_motion("LFT")
                current_status = f"SPIN LEFT @ {speed}"
            elif key == ord('d'):
                send_motion("RGT")
                current_status = f"SPIN RIGHT @ {speed}"
            elif key == ord('z'):
                send_motion("SLFT")
                current_status = f"GENTLE LEFT @ {speed}"
            elif key == ord('c'):
                send_motion("SRGT")
                current_status = f"GENTLE RIGHT @ {speed}"

        # Hold-to-drive: move only while a key is physically held,
        # stop the instant every key is released.
        if HOLD_TO_DRIVE and not is_drowsy:
            order = poll_drive_keys()
            active = KEY_TO_MOTION[order[-1]] if order else None
            if active is not None:
                if active != last_motion or (now - last_heartbeat) >= HEARTBEAT_INTERVAL:
                    send_motion(active)
                    last_heartbeat = now
                    current_status = f"{MOTION_LABEL[active]} @ {speed}"
            elif last_motion is not None:
                last_motion = None
                transmit_command("STOP")
                current_status = "STOPPED"

        # Live HUD
        hud_color = (0, 0, 255) if is_drowsy else (0, 255, 0)
        cv2.putText(frame, f"EAR: {current_ear:.2f}  Speed: {speed}", (25, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
        cv2.putText(frame, f"Status: {current_status}", (25, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, hud_color, 2)
        cv2.putText(frame, f"Radio: {radio_status}", (25, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)

        cv2.imshow("Driver Drowsiness Controller", frame)
        frame_count += 1

        # Allow clean window-close exit
        if cv2.getWindowProperty("Driver Drowsiness Controller", cv2.WND_PROP_VISIBLE) < 1:
            print("[INFO] Window closed by user.")
            transmit_command("STOP")
            break
except KeyboardInterrupt:
    print("\n[INFO] Interrupted (Ctrl+C). Stopping...")
    try:
        transmit_command("STOP")
    except Exception:
        pass
finally:
    cap.release()
    cv2.destroyAllWindows()
    if USE_TASKS_API and landmarker is not None:
        try:
            landmarker.close()
        except Exception:
            pass
    if ser is not None:
        try:
            ser.close()
            print("[OK] Serial closed.")
        except Exception:
            pass
    print(f"[OK] Shutdown after {frame_count} frames.")
