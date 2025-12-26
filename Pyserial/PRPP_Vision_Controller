import tkinter as tk
from tkinter import ttk, messagebox
import threading
import queue
import serial
import serial.tools.list_ports
import time
import numpy as np
import math
from scipy.optimize import minimize
import cv2

BAUDRATE = 115200
REACH_TIMEOUT = 10.0
CAM_INDEX = 1  # 0 for mobile

TARGETS = {
    "RED":    (300, 280, 0),
    "GREEN":  (350, 20, 0),
    "YELLOW": (100, -200, 0),
    "NONE":   None
}

PICK_HOME = (100, 250, 20) 
PICK_HOME1 = (100, 250, 5)   
PICK_APPROACH = (100, 250, 200) 
PLACE_POSITIONS = {
    "RED": {
        "approach": (350, 150, 200),
        "place": (350, 150, 10)
    },
    "GREEN": {
        "approach": (350, 50, 200),
        "place": (350, 50, 10)
    },
    "BLUE": {
        "approach": (620, 0, 200),
        "place": (620, 0, 10)
    },
    "YELLOW": {
        "approach": (-200, 50, 200),
        "place": (-200, 50, 10)
    }
}

task_running = False
stop_task = False
movement_complete = False 
last_feedback = ""  


COLOR_RANGES = {
    "RED": [
        (np.array([0, 120, 70]), np.array([10, 255, 255])),
        (np.array([170, 120, 70]), np.array([180, 255, 255]))
    ],
    "GREEN": [
        (np.array([35, 100, 100]), np.array([85, 255, 255]))
    ],
    "YELLOW": [
        (np.array([20, 100, 100]), np.array([30, 255, 255]))
    ]
}

MIN_AREA = 1000

ser = None
serial_thread_running = True
msg_queue = queue.Queue()
tx_queue = queue.Queue()
last_solution = None
error1 = None
current_solution = None

vision_enabled = False
vision_thread = None
camera_cap = None


def solve_ik_optimized(target_x, target_y, target_z):
    
    try:
        def objective(q):
            d1, theta_rad, d2, d3 = q
            Px = d1 + d3 * np.sin(theta_rad)
            Py = d3 * np.cos(theta_rad)
            Pz = d2
            global error1
            error1 = np.sqrt((Px - target_x)**2 + (Py - target_y)**2 + (Pz - target_z)**2)
            return error1

        global last_solution

        theta_init = np.arctan2(target_x, target_y)
        r_xy = np.sqrt(target_x**2 + target_y**2)
        d3_init = np.clip(r_xy * 0.8, 10, 280)  # Use 80% of distance for better reach
        d1_init = np.clip(target_x - d3_init * np.sin(theta_init), 0, 350)
        d2_init = np.clip(target_z, 0, 275)


        if last_solution is not None:
            last_theta = last_solution[1]
            theta_diff = abs(theta_init - last_theta)
            
            if theta_diff < np.pi/2:
                x0 = last_solution.copy()
                msg_queue.put(f"Inverse Kinematics Using previous solution as initial guess")
            else:
                x0 = [d1_init, theta_init, d2_init, d3_init]
                msg_queue.put(f"Inverse Kinematics Using geometric initial guess")
        else:
            x0 = [d1_init, theta_init, d2_init, d3_init]

        bounds = [
            (0, 350),           # d1: Extended range for far positions
            (-np.pi, np.pi),    # theta: Full rotation range
            (0, 275),           # d2: Z-lift limits
            (0, 280)            # d3: Y-extension limits
        ]

        result = minimize(
            objective,
            x0,
            method='L-BFGS-B',
            bounds=bounds,
            options={'maxiter': 1000, 'ftol': 1e-6}
        )


        if not result.success or result.fun > 1.0:
            msg_queue.put(f"[IK-OPT] Trying SLSQP method")
            result = minimize(
                objective,
                x0,
                method='SLSQP',
                bounds=bounds,
                options={'maxiter': 1000, 'ftol': 1e-6}
            )

        if result.success or result.fun < 5.0:
            d1_opt, theta_rad_opt, d2_opt, d3_opt = result.x
            theta_deg_opt = np.degrees(theta_rad_opt)
            
            while theta_deg_opt > 180:
                theta_deg_opt -= 360
            while theta_deg_opt < -180:
                theta_deg_opt += 360

            final_error = result.fun
            msg_queue.put(f"[IK-OPT] Solution - Error: {final_error:.4f} mm")
            
            if result.fun < 5.0:
                msg_queue.put(f"[IK-OPT] d1={d1_opt:.2f}mm, θ={theta_deg_opt:.2f}°, d2={d2_opt:.2f}mm, d3={d3_opt:.2f}mm")
                last_solution = [d1_opt, theta_rad_opt, d2_opt, d3_opt]
                return (d1_opt, d2_opt, d3_opt, theta_deg_opt)

        msg_queue.put(f"[IK-OPT] Optimization failed: Error={result.fun:.2f}mm")
        return None

    except Exception as e:
        msg_queue.put(f"[IK-OPT] Error: {e}")
        return None



class ColorDetector:
    
    def __init__(self):
        self.cap = None
        self.last_detected = "NONE"
        self.detection_count = 0
        self.confirmation_threshold = 10  
        self.running = False

    def initialize_camera(self):
        try:
            self.cap = cv2.VideoCapture(CAM_INDEX)
            if not self.cap.isOpened():
                msg_queue.put("[VISION] Camera initialization failed!")
                return False
            msg_queue.put("[VISION] Camera initialized successfully")
            return True
        except Exception as e:
            msg_queue.put(f"[VISION] Camera error: {e}")
            return False

    def get_color(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        detected_color = "NONE"
        largest_area = 0
        center = None
        bbox = None

        for color_name, ranges in COLOR_RANGES.items():
            mask = np.zeros(hsv.shape[:2], dtype="uint8")
            
            for (lower, upper) in ranges:
                mask += cv2.inRange(hsv, lower, upper)
            
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                c = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(c)
                
                if area > MIN_AREA and area > largest_area:
                    largest_area = area
                    detected_color = color_name
                    x, y, w, h = cv2.boundingRect(c)
                    bbox = (x, y, w, h)
                    center = (x + w//2, y + h//2)

        return detected_color, center, bbox

    def process_detection(self, color):

        if color == self.last_detected and color != "NONE":
            self.detection_count += 1
        else:
            self.detection_count = 0
            self.last_detected = color
        
        return self.detection_count > self.confirmation_threshold

    def release(self):
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()

detector = ColorDetector()


def move_robot_to(target_x, target_y, target_z, mode=1):

    msg_queue.put(f"[VISION] Moving to target: X={target_x}, Y={target_y}, Z={target_z}")
    
    result = solve_ik_optimized(target_x, target_y, target_z)
    
    if result:
        d1, d2, d3, theta_deg = result
        
        if error1 is not None and error1 > 5.0:
            msg_queue.put(f"[WARNING] Error {error1:.2f}mm > 5mm. Movement cancelled.")
            return False
        
        send_position_command(d1, d2, d3, theta_deg, mode)
        return True
    else:
        msg_queue.put("[VISION] IK failed for target position")
        return False

def send_position_command(d1, d2, d3, th_deg, mode):
    global movement_complete
    
    if not all(isinstance(val, (int, float)) for val in [d1, d2, d3, th_deg, mode]):
        msg_queue.put("[ERR] Invalid numeric types in command.")
        return
    
    cmd = f"POS:{int(d1)},{int(d2)},{int(d3)},{int(th_deg)},{int(mode)}\n"
    
    try:
        movement_complete = False  
        tx_queue.put(cmd)
        msg_queue.put(f"[CMD] X={int(d1)}, Z={int(d2)}, Y={int(d3)}, R={int(th_deg)}°, Mode={int(mode)}")
    except Exception as e:
        msg_queue.put(f"[ERR] Queue failed: {e}")

def serial_reader():
    global ser, serial_thread_running, movement_complete, last_feedback
    while serial_thread_running:
        if ser and ser.is_open:
            try:
                line = ser.readline().decode(errors='ignore').strip()
                if line:
                    msg_queue.put(f"[RX] {line}")
                    line_upper = line.upper()
                    if "REACHED" in line_upper:
                        movement_complete = True
                        msg_queue.put("[FEEDBACK] Movement REACHED target")
                    elif "MOVING" in line_upper:
                        movement_complete = False
                        msg_queue.put("[FEEDBACK] Motors MOVING")
                    elif "ERROR" in line_upper or "FAIL" in line_upper:
                        movement_complete = True 
                        msg_queue.put(f"[FEEDBACK] Error detected: {line}")
                    last_feedback = line
            except serial.SerialException:
                msg_queue.put("[ERR] Serial disconnected")
                try:
                    ser.close()
                except:
                    pass
                ser = None
            except Exception:
                pass
        time.sleep(0.01)

def serial_writer():
    global ser, serial_thread_running
    while serial_thread_running:
        try:
            cmd = tx_queue.get(timeout=0.1)
        except queue.Empty:
            continue

        if ser and ser.is_open:
            try:
                ser.write(cmd.encode())
                msg_queue.put(f"[TX] {cmd.strip()}")
            except Exception as e:
                msg_queue.put(f"[TX-Error] {e}")
        else:
            msg_queue.put("[TX] Serial not open.")
            time.sleep(0.5)


def wait_for_robot_ready(timeout=20):
    global movement_complete
    
    movement_complete = False
    start_time = time.time()
    
    msg_queue.put(f"[WAIT] Waiting for movement completion (timeout: {timeout}s)")
    
    while time.time() - start_time < timeout:
        if movement_complete:
            elapsed = time.time() - start_time
            msg_queue.put(f"[WAIT] Movement confirmed in {elapsed:.2f}s")
            return True
        time.sleep(0.1) 
    
    msg_queue.put(f"[WAIT]TIMEOUT after {timeout}s - Movement not confirmed!")
    return False

def activate_magnet(on=True, delay=0.5):
    cmd = f"MAG:{1 if on else 0}\n"
    try:
        tx_queue.put(cmd)
        state = "ON" if on else "OFF"
        msg_queue.put(f"[MAGNET] {state}")
        time.sleep(delay)
    except Exception as e:
        msg_queue.put(f"[MAGNET-ERR] {e}")

def detect_color_once(show_preview=True, preview_duration=3.0):
    if not detector.cap or not detector.cap.isOpened():
        if not detector.initialize_camera():
            return "NONE"
    
    detected_colors = []
    start_time = time.time()
    frame_count = 0
    
    msg_queue.put(f"[VISION] Starting color detection (showing preview for {preview_duration}s)...")
    
    while time.time() - start_time < preview_duration:
        ret, frame = detector.cap.read()
        if ret:
            frame = cv2.flip(frame, 1)
            color, center, bbox = detector.get_color(frame)
            
            if color != "NONE":
                detected_colors.append(color)
            
            if show_preview:
                if bbox:
                    x, y, w, h = bbox
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 3)
                    cv2.putText(frame, f"DETECTED: {color}", (x, y-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                            
                status = f"Scanning [{frame_count} frames]"
                if detected_colors:
                    most_common = max(set(detected_colors), key=detected_colors.count)
                    count = detected_colors.count(most_common)
                    status = f"LIKELY: {most_common} ({count}/{len(detected_colors)} frames)"
                
                cv2.putText(frame, status, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                remaining = preview_duration - (time.time() - start_time)
                cv2.putText(frame, f"Time: {remaining:.1f}s", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                h, w = frame.shape[:2]
                cv2.line(frame, (w//2, 0), (w//2, h), (50, 50, 50), 1)
                cv2.line(frame, (0, h//2), (w, h//2), (50, 50, 50), 1)
                
                cv2.imshow("Color Detection - Task Sequence", frame)

                if cv2.waitKey(30) & 0xFF == ord('q'):
                    msg_queue.put("[VISION] Detection cancelled by user")
                    break
            
            frame_count += 1
        time.sleep(0.05)
    
    if show_preview:
        cv2.destroyWindow("Color Detection - Task Sequence")
    
    if detected_colors:
        final_color = max(set(detected_colors), key=detected_colors.count)
        confidence = detected_colors.count(final_color) / len(detected_colors) * 100
        msg_queue.put(f"[VISION] Detected: {final_color} (confidence: {confidence:.1f}%, {len(detected_colors)} samples)")
        return final_color
    
    msg_queue.put(f"[VISION] No color detected in {frame_count} frames")
    return "NONE"

def stop_task_sequence():

    global stop_task, task_running
    if task_running:
        stop_task = True
        msg_queue.put("[TASK] STOP REQUESTED - Halting sequence...")
        activate_magnet(False) 
        messagebox.showwarning("Task Stopped", "Task sequence stopped by user")
    else:
        msg_queue.put("[TASK] No task running")

def task_space_sequence():
    global task_running, stop_task
    
    if not ser or not ser.is_open:
        msg_queue.put("[TASK] ERROR: Serial not connected!")
        messagebox.showerror("Not Connected", "Connect to robot first")
        return
    
    task_running = True
    stop_task = False
    cycle_count = 0
    msg_queue.put("[TASK] STARTING CONTINUOUS TASK SEQUENCE ")   
    try:
        msg_queue.put("[TASK] Step 1: Moving to initial home position")
        if not move_robot_to(PICK_HOME[0], PICK_HOME[1], PICK_HOME[2], mode=1):
            msg_queue.put("[TASK] ERROR: IK failed for home position")
            messagebox.showerror("Task Failed", "Failed to move to home position")
            task_running = False
            return
        if not wait_for_robot_ready(timeout=40):
            msg_queue.put("[TASK] ERROR: Home movement timeout")
            messagebox.showerror("Task Failed", "Robot didn't reach home position")
            task_running = False
            return
        msg_queue.put("[TASK] Robot at home position Ready to start cycles")
    except Exception as e:
        msg_queue.put(f"[TASK] ERROR: Failed to reach home - {e}")
        messagebox.showerror("Task Failed", f"Initial positioning failed: {e}")
        task_running = False
        return
    
    while not stop_task:
        cycle_count += 1
        msg_queue.put(f"\n[TASK] CYCLE {cycle_count} START")
        
        try:

            msg_queue.put("[TASK] Step 2: Moving to approach height")
            if not move_robot_to(PICK_APPROACH[0], PICK_APPROACH[1], PICK_APPROACH[2], mode=1):
                raise Exception("IK failed for approach")
            if not wait_for_robot_ready(timeout=40):
                raise Exception("Approach movement timeout")
            if stop_task:
                raise Exception("Stopped by user")

            msg_queue.put("[TASK] Step 3: Waiting for object detection...")
            detected_color = "NONE"
            detection_attempts = 0
            
            while detected_color == "NONE":
                if stop_task:
                    raise Exception("Stopped by user")
                
                detection_attempts += 1
                msg_queue.put(f"[TASK] Detection attempt #{detection_attempts} - Scanning for colored plate...")
                detected_color = detect_color_once(show_preview=True, preview_duration=5.0)
                msg_queue.put(f"[TASK] Result: {detected_color}")
                
                if detected_color == "NONE":
                    msg_queue.put("[TASK] No object detected. Waiting 3 seconds before retry...")
                    time.sleep(3) 
                    if stop_task:
                        raise Exception("Stopped by user")
            
            msg_queue.put(f"[TASK] Object detected: {detected_color} (after {detection_attempts} attempt(s))")
            
            if detected_color not in PLACE_POSITIONS:
                msg_queue.put(f"[TASK] WARNING: Unknown color {detected_color}. Using RED as default.")
                detected_color = "RED"

            msg_queue.put("[TASK] Step 4: Lowering to pick position")
            if not move_robot_to(PICK_HOME1[0], PICK_HOME1[1], PICK_HOME1[2], mode=1):
                raise Exception("IK failed for pick position")
            if not wait_for_robot_ready(timeout=40):
                raise Exception("Pick lowering timeout")
            if stop_task:
                raise Exception("Stopped by user")
            

            msg_queue.put("[TASK] Step 5: Activating magnet (PICK)")
            activate_magnet(True, delay=3)
            if stop_task:
                activate_magnet(False)
                raise Exception("Stopped by user")
            

            msg_queue.put("[TASK] Step 6: Lifting with object")
            if not move_robot_to(PICK_APPROACH[0], PICK_APPROACH[1], PICK_APPROACH[2], mode=1):
                activate_magnet(False)
                raise Exception("IK failed for lift")
            if not wait_for_robot_ready(timeout=40):
                activate_magnet(False)
                raise Exception("Lift movement timeout")
            if stop_task:
                activate_magnet(False)
                raise Exception("Stopped by user")
            
            place_pos = PLACE_POSITIONS[detected_color]
            msg_queue.put(f"[TASK] Step 7: Moving to {detected_color} place approach position")
            approach_pos = place_pos["approach"]
            if not move_robot_to(approach_pos[0], approach_pos[1], approach_pos[2], mode=1):
                activate_magnet(False)
                raise Exception("IK failed for place approach")
            if not wait_for_robot_ready(timeout=25):
                activate_magnet(False)
                raise Exception("Place approach movement timeout")
            if stop_task:
                activate_magnet(False)
                raise Exception("Stopped by user")
            
            msg_queue.put("[TASK] Step 8: Lowering to place position")
            place_final = place_pos["place"]
            if not move_robot_to(place_final[0], place_final[1], place_final[2], mode=1):
                activate_magnet(False)
                raise Exception("IK failed for place position")
            if not wait_for_robot_ready(timeout=40):
                activate_magnet(False)
                raise Exception("Place lowering timeout")
            if stop_task:
                activate_magnet(False)
                raise Exception("Stopped by user")

            msg_queue.put("[TASK] Step 9: Deactivating magnet (RELEASE)")
            activate_magnet(False, delay=2)
            if stop_task:
                raise Exception("Stopped by user")
            
            msg_queue.put("[TASK] Step 10: Lifting from place position")
            if not move_robot_to(approach_pos[0], approach_pos[1], approach_pos[2], mode=1):
                raise Exception("IK failed for lift from place")
            if not wait_for_robot_ready(timeout=40):
                raise Exception("Lift from place timeout")
            if stop_task:
                raise Exception("Stopped by user")

            msg_queue.put("[TASK] Step 11: Returning to home")
            if not move_robot_to(PICK_HOME[0], PICK_HOME[1], PICK_HOME[2], mode=1):
                raise Exception("IK failed for return home")
            if not wait_for_robot_ready(timeout=40):
                raise Exception("Return home timeout")
            if stop_task:
                raise Exception("Stopped by user")
            
            msg_queue.put(f"[TASK] CYCLE {cycle_count} COMPLETED ")
            msg_queue.put(f"[TASK] {detected_color} plate placed successfully!")
            msg_queue.put("[TASK] Waiting 2 seconds before next cycle...")
            time.sleep(2) 
            
        except Exception as e:
            msg_queue.put(f"[TASK] ERROR in cycle #{cycle_count}: {e}")
            msg_queue.put("[TASK] Initiating emergency stop")
            
            activate_magnet(False)
            time.sleep(0.5)
            
            try:
                msg_queue.put("[TASK] Attempting emergency return to home...")
                if move_robot_to(PICK_HOME[0], PICK_HOME[1], PICK_HOME[2], mode=1):
                    wait_for_robot_ready(timeout=10)
                    msg_queue.put("[TASK] Emergency home completed")
            except:
                msg_queue.put("[TASK] Emergency home failed - manual intervention may be needed")
            
            if not stop_task:
                messagebox.showerror("Task Failed", f"Cycle #{cycle_count} failed: {e}\n\nRobot returned to safe position.\n\nTask sequence stopped.")
            
            break 
    
    task_running = False
    stop_task = False
    msg_queue.put(f"[TASK]TASK SEQUENCE ENDED - Completed {cycle_count} cycle")

def start_task_sequence():
    if task_running:
        messagebox.showwarning("Task Running", "A task is already in progress")
        return
    
    task_thread = threading.Thread(target=task_space_sequence, daemon=True)
    task_thread.start()

def vision_loop():
    global vision_enabled, detector
    
    if not detector.initialize_camera():
        msg_queue.put("[VISION] Failed to start vision system")
        return
    
    msg_queue.put("[VISION] Vision system started - Press 'q' in camera window to stop")
    
    while vision_enabled:
        ret, frame = detector.cap.read()
        if not ret:
            msg_queue.put("[VISION] Failed to read frame")
            time.sleep(0.1)
            continue
        
        frame = cv2.flip(frame, 1)

        color, center, bbox = detector.get_color(frame)
        
        if detector.process_detection(color):
            target_coords = TARGETS.get(color)
            
            if target_coords:
                status_text = f"TARGET: {color} -> {target_coords}"
                
                if ser and ser.is_open:
                    mode = 1
                    move_robot_to(target_coords[0], target_coords[1], target_coords[2], mode)
                else:
                    msg_queue.put("[VISION] Serial not connected - cannot move robot")
                
                if bbox:
                    x, y, w, h = bbox
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    cv2.putText(frame, color, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        else:
            status_text = "SCANNING"
        
        cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        h, w = frame.shape[:2]
        cv2.line(frame, (w//2, 0), (w//2, h), (50, 50, 50), 1)
        cv2.line(frame, (0, h//2), (w, h//2), (50, 50, 50), 1)
        
        cv2.imshow("Robot Vision Eye", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            msg_queue.put("[VISION] Stopping vision system")
            break
    
    detector.release()
    msg_queue.put("[VISION] Vision system stopped")



def refresh_ports():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    combo_ports['values'] = ports
    if ports:
        combo_ports.set(ports[0])
    else:
        combo_ports.set("")

def toggle_serial():
    global ser
    if ser and ser.is_open:
        try:
            ser.close()
        except Exception as e:
            msg_queue.put(f"[ERR] closing port: {e}")
        ser = None
        btn_connect.config(text="Connect")
        combo_ports.config(state="readonly")
        btn_refresh.config(state="normal")
        msg_queue.put("[SYS] Disconnected")
    else:
        port = combo_ports.get()
        if not port:
            msg_queue.put("[ERR] No port selected")
            return
        try:
            ser = serial.Serial(port, BAUDRATE, timeout=0.1)
            time.sleep(2)
            btn_connect.config(text="Disconnect")
            combo_ports.config(state="disabled")
            btn_refresh.config(state="disabled")
            msg_queue.put(f"[SYS] Connected to {port}")
        except Exception as e:
            ser = None
            msg_queue.put(f"[ERR] Open port failed: {e}")

def toggle_vision():
    """Start/stop vision system"""
    global vision_enabled, vision_thread
    
    if vision_enabled:
        vision_enabled = False
        if vision_thread:
            vision_thread.join(timeout=2.0)
        btn_vision.config(text="Start Vision")
        msg_queue.put("[VISION] Stopped")
    else:
        vision_enabled = True
        vision_thread = threading.Thread(target=vision_loop, daemon=True)
        vision_thread.start()
        btn_vision.config(text="Stop Vision")
        msg_queue.put("[VISION] Started")

def on_solve_ik():
    try:
        x = float(entry_x.get())
        y = float(entry_y.get())
        z = float(entry_z.get())
        
        msg_queue.put(f"[IK] Solving for: X={x}, Y={y}, Z={z} mm")
        result = solve_ik_optimized(x, y, z)
        
        if result:
            d1, d2, d3, theta_deg = result
            
            lbl_d1_result.config(text=f"{d1:.2f} mm")
            lbl_theta_result.config(text=f"{theta_deg:.2f}°")
            lbl_d2_result.config(text=f"{d2:.2f} mm")
            lbl_d3_result.config(text=f"{d3:.2f} mm")
            
            global current_solution
            current_solution = (d1, d2, d3, theta_deg)
            
            if error1 is not None and error1 > 5.0:
                btn_send.config(state="disabled")
                msg_queue.put(f"[WARNING] Error {error1:.2f}mm > 5mm. Send disabled.")
                messagebox.showwarning("High Error", f"IK error: {error1:.2f} mm (> 5mm threshold)")
            else:
                btn_send.config(state="normal")
        else:
            messagebox.showerror("IK Failed", "Could not find solution")
            btn_send.config(state="disabled")
            
    except ValueError:
        messagebox.showerror("Input Error", "Please enter valid numeric values")

def on_send_to_robot():
    """Send current solution to robot"""
    global current_solution
    
    if current_solution is None:
        messagebox.showwarning("No Solution", "Solve IK first")
        return
    
    if not ser or not ser.is_open:
        messagebox.showwarning("Not Connected", "Connect to serial port first")
        return
    
    if error1 is not None and error1 > 5.0:
        messagebox.showwarning("Error Too High", f"Error: {error1:.2f} mm")
        return
    
    d1, d2, d3, theta_deg = current_solution
    mode = int(combo_mode.get())
    send_position_command(d1, d2, d3, theta_deg, mode)

def on_home():
    if not ser or not ser.is_open:
        messagebox.showwarning("Not Connected", "Connect to serial port first")
        return
    
    mode = int(combo_mode.get())
    send_position_command(0, 0, 0, 0, mode)
    msg_queue.put("[CMD] Homing robot")

def send_mode_command(mode):

    if not isinstance(mode, (int, float)) or mode not in [1, 2]:
        msg_queue.put("[ERR] Mode must be 1 or 2")
        return  
    cmd = f"MODE:{int(mode)}\n"  
    try:
        tx_queue.put(cmd)
        msg_queue.put(f"[CMD] Mode changed to: {int(mode)}")
    except Exception as e:
        msg_queue.put(f"[ERR] Queue failed: {e}")

def update_gui():
    while not msg_queue.empty():
        msg = msg_queue.get_nowait()
        txt_log.insert(tk.END, msg + "\n")
        txt_log.see(tk.END)

        if "[SYS] Disconnected" in msg:
            btn_connect.config(text="Connect")
            combo_ports.config(state="readonly")
            btn_refresh.config(state="normal")
    
    root.after(100, update_gui)

def on_closing():
    global serial_thread_running, vision_enabled
    
    if vision_enabled:
        vision_enabled = False
        time.sleep(0.5)

    serial_thread_running = False
    time.sleep(0.3)
    
    if ser and ser.is_open:
        ser.close()
    
    root.destroy()

root = tk.Tk()
root.title("PRPP Robot Vision Controller")
root.protocol("WM_DELETE_WINDOW", on_closing)


frame_serial = ttk.LabelFrame(root, text="Serial Connection", padding=10)
frame_serial.grid(row=0, column=0, padx=10, pady=10, sticky="ew")

combo_ports = ttk.Combobox(frame_serial, state="readonly", width=15)
combo_ports.grid(row=0, column=0, padx=5)

btn_refresh = ttk.Button(frame_serial, text="Refresh", command=refresh_ports)
btn_refresh.grid(row=0, column=1, padx=5)

btn_connect = ttk.Button(frame_serial, text="Connect", command=toggle_serial)
btn_connect.grid(row=0, column=2, padx=5)

frame_vision = ttk.LabelFrame(root, text="Vision System", padding=10)
frame_vision.grid(row=1, column=0, padx=10, pady=10, sticky="ew")

btn_vision = ttk.Button(frame_vision, text="Start Vision", command=toggle_vision, width=20)
btn_vision.grid(row=0, column=0, padx=5, pady=5)

lbl_vision_info = ttk.Label(frame_vision, text="Color detection → Auto movement", foreground="blue")
lbl_vision_info.grid(row=1, column=0, padx=5)

btn_task_sequence = ttk.Button(frame_vision, text="Start Pick & Place Task", command=start_task_sequence, width=20)
btn_task_sequence.grid(row=2, column=0, padx=5, pady=5)
btn_stop_task = ttk.Button(frame_vision, text="STOP Task", command=stop_task_sequence, width=20)
btn_stop_task.grid(row=3, column=0, padx=5, pady=5)
btn_stop_task.config(style='Danger.TButton')  # Red style

lbl_task_info = ttk.Label(frame_vision, text="Auto: Detect→Pick→Place by color", foreground="green")
lbl_task_info.grid(row=4, column=0, padx=5)

frame_ik = ttk.LabelFrame(root, text="Manual IK Control", padding=10)
frame_ik.grid(row=2, column=0, padx=10, pady=10, sticky="ew")

ttk.Label(frame_ik, text="Target X (mm):").grid(row=0, column=0, sticky="w")
entry_x = ttk.Entry(frame_ik, width=10)
entry_x.grid(row=0, column=1, padx=5)
entry_x.insert(0, "200")

ttk.Label(frame_ik, text="Target Y (mm):").grid(row=1, column=0, sticky="w")
entry_y = ttk.Entry(frame_ik, width=10)
entry_y.grid(row=1, column=1, padx=5)
entry_y.insert(0, "150")

ttk.Label(frame_ik, text="Target Z (mm):").grid(row=2, column=0, sticky="w")
entry_z = ttk.Entry(frame_ik, width=10)
entry_z.grid(row=2, column=1, padx=5)
entry_z.insert(0, "50")

btn_solve = ttk.Button(frame_ik, text="Solve IK", command=on_solve_ik)
btn_solve.grid(row=3, column=0, columnspan=2, pady=10)

frame_results = ttk.LabelFrame(root, text="IK Solution", padding=10)
frame_results.grid(row=3, column=0, padx=10, pady=10, sticky="ew")

ttk.Label(frame_results, text="d1 (X-Slide):").grid(row=0, column=0, sticky="w")
lbl_d1_result = ttk.Label(frame_results, text="-- mm", foreground="blue")
lbl_d1_result.grid(row=0, column=1, sticky="w")

ttk.Label(frame_results, text="θ (Rotation):").grid(row=1, column=0, sticky="w")
lbl_theta_result = ttk.Label(frame_results, text="-- °", foreground="blue")
lbl_theta_result.grid(row=1, column=1, sticky="w")

ttk.Label(frame_results, text="d2 (Z-Lift):").grid(row=2, column=0, sticky="w")
lbl_d2_result = ttk.Label(frame_results, text="-- mm", foreground="blue")
lbl_d2_result.grid(row=2, column=1, sticky="w")

ttk.Label(frame_results, text="d3 (Y-Extension):").grid(row=3, column=0, sticky="w")
lbl_d3_result = ttk.Label(frame_results, text="-- mm", foreground="blue")
lbl_d3_result.grid(row=3, column=1, sticky="w")

frame_move = ttk.LabelFrame(root, text="Movement", padding=10)
frame_move.grid(row=4, column=0, padx=10, pady=10, sticky="ew")

ttk.Label(frame_move, text="Mode:").grid(row=0, column=0, sticky="w")
combo_mode = ttk.Combobox(frame_move, values=["1", "2"], state="readonly", width=8)
combo_mode.set("1")
combo_mode.grid(row=0, column=1, padx=5)

btn_send_mode = ttk.Button(frame_serial, text="Send Mode", command=lambda: send_mode_command(int(combo_mode.get())))
btn_send_mode.grid(row=1, column=3, padx=5)

btn_send = ttk.Button(frame_move, text="Send to Robot", command=on_send_to_robot, state="disabled")
btn_send.grid(row=2, column=0, columnspan=2, pady=5)

btn_home = ttk.Button(frame_move, text="Home Robot", command=on_home)
btn_home.grid(row=3, column=0, columnspan=2, pady=5)


frame_log = ttk.LabelFrame(root, text="System Log", padding=10)
frame_log.grid(row=0, column=1, rowspan=5, padx=10, pady=10, sticky="nsew")

txt_log = tk.Text(frame_log, width=60, height=35, wrap=tk.WORD)
txt_log.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

scroll = ttk.Scrollbar(frame_log, command=txt_log.yview)
scroll.pack(side=tk.RIGHT, fill=tk.Y)
txt_log.config(yscrollcommand=scroll.set)

root.columnconfigure(1, weight=1)
root.rowconfigure(0, weight=1)

reader_thread = threading.Thread(target=serial_reader, daemon=True)
reader_thread.start()

writer_thread = threading.Thread(target=serial_writer, daemon=True)
writer_thread.start()

refresh_ports()
root.after(100, update_gui)

msg_queue.put("=== PRPP Vision Controller Started ===")
msg_queue.put("1. Connect to ESP32")
msg_queue.put("2. Start Vision for auto detection")
msg_queue.put("3. Or use Manual IK for precise control")
root.mainloop()
