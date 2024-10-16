import cv2
import mediapipe as mp
import numpy as np
import time
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk
import threading
import csv
import warnings
import sys
import os
import serial
import logging

warnings.filterwarnings("ignore", category=UserWarning)
sys.stderr = open(os.devnull, 'w')

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
logging.getLogger('tensorflow').setLevel(logging.FATAL)
logging.getLogger('mediapipe').setLevel(logging.ERROR)

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

class HandGestureApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Hand Gesture Recognition")

        self.max_num_hands = 2
        self.min_detection_confidence = tk.DoubleVar(value=0.7)
        self.min_tracking_confidence = tk.DoubleVar(value=0.7)
        self.gesture_log = []
        self.last_logged_gesture = {}
        self.last_log_time = {}

        self.update_settings_job = None 

        self.buffered_gestures = [
            "Gesture: Click Detected",
            "Gesture: Pinch Detected",
            "Gesture: Swipe Left",
            "Gesture: Swipe Right"
        ]

        self.gesture_buffers = {gesture: 0 for gesture in self.buffered_gestures}
        self.required_buffer_size = 3 

        self.moving_average_size = 5
        self.thumb_index_distances = {idx: [] for idx in range(self.max_num_hands)}
        self.hand_movements = {idx: [] for idx in range(self.max_num_hands)}

        self.gesture_states = {gesture: "Idle" for gesture in self.buffered_gestures}

        self.create_widgets()

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            messagebox.showerror("Error", "Cannot access webcam")
            self.running = False
            self.cap.release()
            self.root.destroy()
            return
        self.running = True

        self.prev_time = {}
        self.prev_thumb_index_distance = {}
        self.prev_index_tip_y = {}
        self.prev_hand_x = {}
        self.swipe_gesture = {}
        self.swipe_start_time = {}

        self.hands = mp_hands.Hands(
            max_num_hands=self.max_num_hands,
            min_detection_confidence=self.min_detection_confidence.get(),
            min_tracking_confidence=self.min_tracking_confidence.get()
        )

        try:
            self.serial_port = serial.Serial('COM3', 9600, timeout=1)
            time.sleep(2)
            print("Serial port opened successfully.")
        except Exception as e:
            messagebox.showwarning("Warning", f"Could not open serial port: {e}\nThe program will continue without ESP32 connection.")
            self.serial_port = None

        self.video_loop()

    def create_widgets(self):
        self.video_label = tk.Label(self.root)
        self.video_label.pack()

        settings_frame = tk.Frame(self.root)
        settings_frame.pack()

        tk.Label(settings_frame, text="Detection Confidence").grid(row=0, column=0)
        detection_slider = ttk.Scale(settings_frame, from_=0.0, to=1.0, orient=tk.HORIZONTAL,
                                     variable=self.min_detection_confidence, command=self.update_settings)
        detection_slider.grid(row=0, column=1)

        tk.Label(settings_frame, text="Tracking Confidence").grid(row=1, column=0)
        tracking_slider = ttk.Scale(settings_frame, from_=0.0, to=1.0, orient=tk.HORIZONTAL,
                                    variable=self.min_tracking_confidence, command=self.update_settings)
        tracking_slider.grid(row=1, column=1)

        log_frame = tk.Frame(self.root)
        log_frame.pack()

        self.log_text = tk.Text(log_frame, height=10, width=50)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        scrollbar = tk.Scrollbar(log_frame, command=self.log_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text['yscrollcommand'] = scrollbar.set

        save_button = tk.Button(self.root, text="Save Log", command=self.save_log)
        save_button.pack()

    def update_settings(self, event=None):
        if self.update_settings_job:
            self.root.after_cancel(self.update_settings_job)
        self.update_settings_job = self.root.after(200, self.apply_settings)

    def apply_settings(self):
        try:
            if hasattr(self, 'hands') and self.hands:
                self.hands.close()
            self.hands = mp_hands.Hands(
                max_num_hands=self.max_num_hands,
                min_detection_confidence=self.min_detection_confidence.get(),
                min_tracking_confidence=self.min_tracking_confidence.get()
            )
            for idx in self.thumb_index_distances:
                self.thumb_index_distances[idx].clear()
            for idx in self.hand_movements:
                self.hand_movements[idx].clear()
            for gesture in self.gesture_buffers:
                self.gesture_buffers[gesture] = 0
            for gesture in self.gesture_states:
                self.gesture_states[gesture] = "Idle"
        except Exception as e:
            messagebox.showerror("Error", f"Failed to update settings: {e}")
        finally:
            self.update_settings_job = None

    def calculate_thresholds(self):
        detection_conf = self.min_detection_confidence.get()
        tracking_conf = self.min_tracking_confidence.get()
        
        pinch_threshold = 0.2 * detection_conf 
        click_diff_threshold = 0.015 * tracking_conf
        swipe_movement_threshold = 0.05 * tracking_conf
        
        return pinch_threshold, click_diff_threshold, swipe_movement_threshold

    def video_loop(self):
        if not self.running:
            return

        ret, frame = self.cap.read()
        if not ret:
            self.running = False
            self.cap.release()
            return

        frame = cv2.flip(frame, 1)

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        result = self.hands.process(rgb_frame)

        if result.multi_hand_landmarks:
            for idx, (hand_landmarks, handedness) in enumerate(
                zip(result.multi_hand_landmarks, result.multi_handedness)
            ):
                mp_drawing.draw_landmarks(
                    frame, hand_landmarks, mp_hands.HAND_CONNECTIONS
                )

                hand_label = handedness.classification[0].label
                (
                    base_gesture,
                    fingers,
                    hand_orientation_deg,
                    thumb_index_distance,
                    hand_size,
                ) = self.recognize_gesture(
                    hand_landmarks.landmark, hand_label
                )

                current_time = time.time()

                final_gesture = base_gesture

                pinch_detected = False
                pinch_threshold, click_diff_threshold, swipe_movement_threshold = self.calculate_thresholds()

                if idx in self.prev_thumb_index_distance:
                    distance_diff = (
                        self.prev_thumb_index_distance[idx] - thumb_index_distance
                    )
                    time_diff = current_time - self.prev_time.get(
                        idx, current_time
                    )
                    if distance_diff > hand_size * 0.1 and time_diff < 0.3:
                        final_gesture = "Gesture: Pinch Detected"
                        pinch_detected = True

                self.prev_thumb_index_distance[idx] = thumb_index_distance
                self.prev_time[idx] = current_time

                click_detected = False
                if not pinch_detected and base_gesture.startswith("Gesture: Pointing"):
                    index_tip_y = hand_landmarks.landmark[
                        mp_hands.HandLandmark.INDEX_FINGER_TIP
                    ].y

                    if idx in self.prev_index_tip_y:
                        diff = self.prev_index_tip_y[idx] - index_tip_y
                        time_diff = current_time - self.prev_time.get(
                            idx, current_time
                        )

                        if diff > click_diff_threshold and time_diff < 0.3:
                            final_gesture = "Gesture: Click Detected"
                            click_detected = True

                    self.prev_index_tip_y[idx] = index_tip_y
                    self.prev_time[idx] = current_time
                else:
                    self.prev_index_tip_y.pop(idx, None)

                if not pinch_detected and not click_detected:
                    hand_x = hand_landmarks.landmark[
                        mp_hands.HandLandmark.WRIST
                    ].x

                    movement = 0
                    if idx in self.prev_hand_x:
                        movement = hand_x - self.prev_hand_x[idx]
                        self.hand_movements[idx].append(movement)
                        if len(self.hand_movements[idx]) > self.moving_average_size:
                            self.hand_movements[idx].pop(0)
                        avg_movement = np.mean(self.hand_movements[idx])

                        if abs(avg_movement) > swipe_movement_threshold:
                            if avg_movement > 0:
                                final_gesture = "Gesture: Swipe Right"
                            else:
                                final_gesture = "Gesture: Swipe Left"
                    self.prev_hand_x[idx] = hand_x

                display_text = final_gesture

                if final_gesture in self.buffered_gestures:
                    self.update_gesture_buffers_and_states(final_gesture)
                    if self.gesture_states[final_gesture] == "Confirmed":
                        self.send_command_to_esp32(final_gesture)
                        self.gesture_states[final_gesture] = "Idle"
                else:
                    self.send_command_to_esp32(final_gesture)

                cv2.putText(
                    frame,
                    f"Hand {idx+1} ({hand_label}): {display_text}",
                    (10, 50 + idx * 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (255, 0, 0),
                    2,
                    cv2.LINE_AA,
                )

                self.log_gesture(idx, hand_label, display_text)

        else:
            self.prev_index_tip_y.clear()
            self.prev_thumb_index_distance.clear()
            self.prev_time.clear()
            self.prev_hand_x.clear()
            self.swipe_gesture.clear()
            self.swipe_start_time.clear()

        cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
        img = Image.fromarray(cv2image)
        imgtk = ImageTk.PhotoImage(image=img)

        self.video_label.imgtk = imgtk
        self.video_label.configure(image=imgtk)

        self.root.after(10, self.video_loop)

    def update_gesture_buffers_and_states(self, gesture):
        if gesture in self.gesture_buffers:
            self.gesture_buffers[gesture] += 1
            if self.gesture_buffers[gesture] >= self.required_buffer_size and self.gesture_states[gesture] == "Idle":
                self.gesture_states[gesture] = "Confirmed"
                self.gesture_buffers[gesture] = 0
        else:
            for g in self.gesture_buffers:
                self.gesture_buffers[g] = 0
                self.gesture_states[g] = "Idle"

    def send_command_to_esp32(self, gesture):
        if getattr(self, 'serial_port', None) is None:
            return 

        gesture_command_map = {
            "Gesture: Open Palm": "CMD_OPEN_PALM",
            "Gesture: Closed Fist": "CMD_CLOSED_FIST",
            "Gesture: Pointing Index": "CMD_POINTING_INDEX",
            "Gesture: Pinch Detected": "CMD_PINCH",
            "Gesture: Click Detected": "CMD_CLICK",
            "Gesture: Swipe Left": "CMD_SWIPE_LEFT",
            "Gesture: Swipe Right": "CMD_SWIPE_RIGHT",
            "Gesture: Pointing Middle": "CMD_POINTING_MIDDLE",
            "Gesture: Pointing Ring": "CMD_POINTING_RING",
            "Gesture: Pointing Pinky": "CMD_POINTING_PINKY",
            "Gesture: Hang Loose": "CMD_HANG_LOOSE",
            "Gesture: Call Me": "CMD_CALL_ME",
            "Gesture: OK Sign": "CMD_OK_SIGN",
            "Gesture: Thumbs Up": "CMD_THUMBS_UP",
            "Gesture: Peace Sign": "CMD_PEACE_SIGN",
            "Gesture: Three Fingers": "CMD_THREE_FINGERS",
            "Gesture: Rock On": "CMD_ROCK_ON",
            "Gesture: Spiderman": "CMD_SPIDERMAN",
            "Gesture: Unknown": "CMD_UNKNOWN"
        }

        command = gesture_command_map.get(gesture, "CMD_UNKNOWN")
        
        try:
            print(f"Sending command to ESP32: {command}")

            self.serial_port.write((command + '\n').encode())

        except serial.SerialException as e:
            print(f"Error sending command: {e}")

    def recognize_gesture(self, landmarks, hand_label):
        fingers = []
        hand_landmarks = mp_hands.HandLandmark

        if hand_label == 'Right':
            if landmarks[hand_landmarks.THUMB_TIP].x < landmarks[hand_landmarks.THUMB_IP].x:
                fingers.append(1) 
            else:
                fingers.append(0)
        else:
            if landmarks[hand_landmarks.THUMB_TIP].x > landmarks[hand_landmarks.THUMB_IP].x:
                fingers.append(1)
            else:
                fingers.append(0)

        for id in [
            hand_landmarks.INDEX_FINGER_TIP,
            hand_landmarks.MIDDLE_FINGER_TIP,
            hand_landmarks.RING_FINGER_TIP,
            hand_landmarks.PINKY_TIP,
        ]:
            if landmarks[id].y < landmarks[id - 2].y:
                fingers.append(1) 
            else:
                fingers.append(0)

        gesture = "Gesture: Unknown"

        wrist = landmarks[hand_landmarks.WRIST]
        index_mcp = landmarks[hand_landmarks.INDEX_FINGER_MCP]
        pinky_mcp = landmarks[hand_landmarks.PINKY_MCP]

        hand_orientation = np.arctan2(
            pinky_mcp.y - index_mcp.y, pinky_mcp.x - index_mcp.x
        )
        hand_orientation_deg = np.degrees(hand_orientation)

        thumb_tip = landmarks[hand_landmarks.THUMB_TIP]
        index_tip = landmarks[hand_landmarks.INDEX_FINGER_TIP]
        thumb_index_distance = np.linalg.norm(
            np.array([thumb_tip.x, thumb_tip.y]) - np.array([index_tip.x, index_tip.y])
        )

        hand_size = np.linalg.norm(
            np.array([wrist.x, wrist.y])
            - np.array(
                [
                    landmarks[hand_landmarks.MIDDLE_FINGER_MCP].x,
                    landmarks[hand_landmarks.MIDDLE_FINGER_MCP].y,
                ]
            )
        )

        if fingers == [1, 1, 1, 1, 1]:
            gesture = "Gesture: Open Palm"
        elif fingers == [0, 0, 0, 0, 0]:
            if thumb_index_distance < hand_size * 0.2:
                gesture = "Gesture: Pinch Detected"
            else:
                gesture = "Gesture: Closed Fist"
        elif fingers == [0, 1, 0, 0, 0]:
            gesture = "Gesture: Pointing Index"
        elif fingers == [0, 0, 1, 0, 0]:
            gesture = "Gesture: Pointing Middle"
        elif fingers == [0, 0, 0, 1, 0]:
            gesture = "Gesture: Pointing Ring"
        elif fingers == [0, 0, 0, 0, 1]:
            gesture = "Gesture: Pointing Pinky"
        elif fingers == [1, 0, 0, 0, 1]:
            if abs(hand_orientation_deg) < 30:
                gesture = "Gesture: Hang Loose"
            else:
                gesture = "Gesture: Call Me"
        elif fingers == [1, 0, 1, 1, 1]:
            if thumb_index_distance < hand_size * 0.2:
                gesture = "Gesture: OK Sign"
            else:
                gesture = "Gesture: Thumbs Up"
        elif fingers == [0, 1, 1, 0, 0]:
            gesture = "Gesture: Peace Sign"
        elif fingers == [0, 1, 1, 1, 0]:
            gesture = "Gesture: Three Fingers"
        elif fingers == [0, 1, 0, 0, 1]:
            gesture = "Gesture: Rock On"
        elif fingers == [1, 1, 0, 0, 1]:
            gesture = "Gesture: Spiderman"
        else:
            gesture = "Gesture: Unknown"

        return gesture, fingers, hand_orientation_deg, thumb_index_distance, hand_size

    def log_gesture(self, idx, hand_label, gesture):
        current_time = time.time()
        last_gesture = self.last_logged_gesture.get(idx)
        last_time = self.last_log_time.get(idx, 0)

        if gesture != last_gesture or (current_time - last_time) >= 1:
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            log_entry = f"{timestamp} - Hand {idx+1} ({hand_label}): {gesture}"
            self.gesture_log.append(log_entry)
            self.log_text.insert(tk.END, log_entry + "\n")
            self.log_text.see(tk.END)

            self.last_logged_gesture[idx] = gesture
            self.last_log_time[idx] = current_time

    def save_log(self):
        with open("gesture_log.csv", "w", newline="") as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(["Timestamp", "Gesture"])
            for entry in self.gesture_log:
                try:
                    timestamp, gesture_info = entry.split(" - ")
                    csv_writer.writerow([timestamp, gesture_info])
                except ValueError:
                    continue
        messagebox.showinfo("Gesture Log", "Gesture log saved to gesture_log.csv")

    def on_closing(self):
        self.running = False
        self.cap.release()
        if getattr(self, 'serial_port', None) is not None:
            self.serial_port.close()
        if hasattr(self, 'hands') and self.hands:
            self.hands.close()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = HandGestureApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
