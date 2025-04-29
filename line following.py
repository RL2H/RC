import cv2
import numpy as np
import time
import RPi.GPIO as GPIO
from collections import deque
from picamera2 import Picamera2

# Motor pin configuration
MOTOR_PINS = {
    'IN1': 24, 'IN2': 23,
    'IN3': 27, 'IN4': 22,
    'ENA': 19, 'ENB': 12
}
PWM_FREQ = 100

# PID constants
KP, KI, KD = 0.5, 0.01, 0.1
pid_integral = 0
pid_last_error = 0

# Speeds
BASE_SPEED = 30
MAX_SPEED = 100
REVERSE_BIAS = 20       # additional speed on one side when reversing
MAX_REVERSE_TIME = 1.5  # seconds
FRAME_RATE = 30         # approximate

# Thresholds
BLACK_AREA_THRESHOLD = 100    # min contour area
MIN_DETECTED_PIXELS = 500     # for switching to shortcut
SHORTCUT_TOP_PIXELS = 100     # for slowdown
NO_LINE_THRESHOLD = 10        # frames before recovery

# Colors
COLOR_RANGES = {
    "blue":   ([100, 120, 70], [130, 255, 255]),
    "green":  ([40, 120, 70],  [80, 255, 255]),
    "yellow": ([20, 120, 70],  [40, 255, 255]),
    "red":    ([0, 120, 70],   [10, 255, 255], [170,120,70],[180,255,255])
}

# ————— EDITED —————
# Now you can list *all* the shortcuts you want to traverse:
SHORTCUT_COLORS = ["yellow", "red"]  
# —————————————————

# GPIO setup
GPIO.setmode(GPIO.BCM)
for pin in MOTOR_PINS.values():
    GPIO.setup(pin, GPIO.OUT)
left_pwm  = GPIO.PWM(MOTOR_PINS['ENA'], PWM_FREQ)
right_pwm = GPIO.PWM(MOTOR_PINS['ENB'], PWM_FREQ)
left_pwm.start(0)
right_pwm.start(0)

def set_motors(ls, rs):
    """Set motor speeds with sign controlling direction pins."""
    ls = max(-MAX_SPEED, min(MAX_SPEED, ls))
    rs = max(-MAX_SPEED, min(MAX_SPEED, rs))
    GPIO.output(MOTOR_PINS['IN1'], ls>0); GPIO.output(MOTOR_PINS['IN2'], ls<0)
    GPIO.output(MOTOR_PINS['IN3'], rs>0); GPIO.output(MOTOR_PINS['IN4'], rs<0)
    left_pwm.ChangeDutyCycle(abs(ls))
    right_pwm.ChangeDutyCycle(abs(rs))

def process_frame(frame):
    """Returns (centroid, line_type, shortcut_ahead, vis_frame)."""
    h, w = frame.shape[:2]
    roi = frame[h//2:, :]  # bottom half

    # 1) Black line detection
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    _, thr = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
    cnts, _ = cv2.findContours(thr, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    black_cx = None
    if cnts:
        c = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(c) > BLACK_AREA_THRESHOLD:
            m = cv2.moments(c)
            if m["m00"]:
                black_cx = int(m["m10"]/m["m00"]) - w//2

    # 2) Shortcut color detection
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    sc_mask = None
    for col, ranges in COLOR_RANGES.items():
        # build mask
        if col == "red":
            l1, u1, l2, u2 = [np.array(r) for r in ranges]
            mask = cv2.inRange(hsv, l1, u1) | cv2.inRange(hsv, l2, u2)
        else:
            l, u = [np.array(r) for r in ranges]
            mask = cv2.inRange(hsv, l, u)

        # find largest contour in that mask
        cnts2, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if cnts2:
            c2 = max(cnts2, key=cv2.contourArea)
            if cv2.contourArea(c2) > BLACK_AREA_THRESHOLD:
                m2 = cv2.moments(c2)
                if m2["m00"]:
                    cx2 = int(m2["m10"]/m2["m00"]) - w//2
                    # ————— EDITED —————
                    # only pick up *any* color in your SHORTCUT_COLORS list:
                    if col in SHORTCUT_COLORS:
                        sc_mask = (mask, cx2)
                    # —————————————————
    # 3) Shortcut ahead detection: top slice
    shortcut_ahead = False
    if sc_mask:
        mask, _cx = sc_mask
        top_slice = mask[0:int(h/4), :]
        if np.count_nonzero(top_slice) > SHORTCUT_TOP_PIXELS:
            shortcut_ahead = True

    # 4) Decide active line
    if sc_mask and shortcut_ahead:
        return sc_mask[1], "shortcut", True, frame
    return black_cx, "black", False, frame

def main():
    global pid_integral, pid_last_error
    picam = Picamera2()
    cfg = picam.create_preview_configuration(main={"size":(320,240),"format":"RGB888"})
    picam.configure(cfg)
    picam.start()

    state = "follow"
    last_centroid = None
    lost_frames = 0
    reverse_start = None
    reverse_dir = None

    dt_target = 1/FRAME_RATE
    prev = time.time()

    while True:
        frame = picam.capture_array()
        c, line_type, ahead, vis = process_frame(frame)

        if c is not None:
            # got line
            last_centroid = c
            lost_frames = 0
            state = "follow"
        else:
            lost_frames += 1
            if lost_frames > NO_LINE_THRESHOLD and state == "follow":
                # enter reverse state
                state = "reverse"
                reverse_start = time.time()
                # pick direction based on last_centroid
                if last_centroid is not None and last_centroid < 0:
                    reverse_dir = "left"
                else:
                    reverse_dir = "right"

        # STATE MACHINE
        if state == "follow" and c is not None:
            # PID line following
            error = c
            pid_integral += error
            derivative = error - pid_last_error
            pid_last_error = error
            corr = KP*error + KI*pid_integral + KD*derivative

            ls = BASE_SPEED - corr
            rs = BASE_SPEED + corr
            set_motors(ls, rs)

        elif state == "reverse":
            elapsed = time.time() - reverse_start
            if elapsed > MAX_REVERSE_TIME:
                state = "follow"
                pid_integral = pid_last_error = 0
            else:
                if reverse_dir == "left":
                    set_motors(-(60+REVERSE_BIAS), -BASE_SPEED)
                else:
                    set_motors(-BASE_SPEED, -(60+REVERSE_BIAS))

        # display
        cv2.putText(vis, f"State:{state}  Line:{line_type}", (10,20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
        cv2.imshow("Feed", vis)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # frame rate cap
        now = time.time()
        dt = now - prev
        if dt < dt_target:
            time.sleep(dt_target - dt)
        prev = now

    # cleanup
    set_motors(0,0)
    left_pwm.stop()
    right_pwm.stop()
    GPIO.cleanup()
    picam.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()