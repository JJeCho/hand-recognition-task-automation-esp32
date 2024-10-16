
# Hand Gesture Recognition System with ESP32 Control

## Overview
While awaiting additional components for this project, I began developing a hand gesture recognition system using Python, OpenCV, MediaPipe, and an ESP32 microcontroller. The system recognizes various hand gestures captured through a webcam and performs actions based on the detected gestures. The ESP32, connected via Wi-Fi, controls an RGB LED, with each gesture mapped to a specific color change. The project aims to explore hand gesture control, improve user interaction, and enhance automation.

## Features
- **Hand Gesture Recognition**: The system detects hand gestures in real-time using OpenCV and MediaPipe.
- **ESP32 Integration**: Based on the recognized gestures, commands are sent to the ESP32 to control an RGB LED's color.
- **Real-time Gesture Logging**: The application allows for logs of recognized gestures to be exported as a CSV file, allowing easy review of actions.
- **Graphical User Interface**: The system includes a Tkinter GUI for setting detection and tracking confidence, and viewing gesture logs.
- **Gesture Buffering**: Gestures like "Click" and "Pinch" use a buffer to minimize false positives.

## Current Gesture Mappings
| Gesture                   | LED Color    | Command Sent to ESP32 |
|----------------------------|--------------|-----------------------|
| Open Palm                  | White        | `CMD_OPEN_PALM`       |
| Closed Fist                | Red          | `CMD_CLOSED_FIST`     |
| Pointing Index             | Green        | `CMD_POINTING_INDEX`  |
| Pinch                      | Blue         | `CMD_PINCH`           |
| Click                      | Yellow       | `CMD_CLICK`           |
| Swipe Left                 | Cyan         | `CMD_SWIPE_LEFT`      |
| Swipe Right                | Magenta      | `CMD_SWIPE_RIGHT`     |
| Pointing Middle            | Orange       | `CMD_POINTING_MIDDLE` |
| Pointing Ring              | Purple       | `CMD_POINTING_RING`   |
| Pointing Pinky             | Pink         | `CMD_POINTING_PINKY`  |
| Hang Loose                 | Lime         | `CMD_HANG_LOOSE`      |
| Call Me                    | Sky Blue     | `CMD_CALL_ME`         |

## Future Improvements
1. **Multithreading**: To improve performance and responsiveness, multithreading should be implemented.
2. **Remote Automation**: Integrate the ESP32 with servos, motors, actuators, and sensors to remotely automate tasks.
   - Collect environmental sensor data with a gesture
   - Turn appliances or lights on or off with a wave
   - Select appliances from a room using swipe, click, or pinch
3. **Improve Data Logging**: Utilize a mongoDB database to log recognized gestures and automated tasks.


## Dependencies
- Python 3.x
- OpenCV
- MediaPipe
- Tkinter
- PIL (Python Imaging Library)
- NumPy
- ESP32 with Arduino IDE support

## Setup Instructions
1. **Clone the repository**:
   ```bash
   git clone https://github.com/your-username/hand-gesture-esp32.git
   ```
2. **Install Python dependencies**:
   ```bash
   pip install opencv-python mediapipe numpy pillow
   ```
3. **Upload the ESP32 code**:
   - Connect the ESP32 to your computer.
   - Open the Arduino IDE and upload the `esp32_rgb_control.ino` sketch to the ESP32.

4. **Run the Python script**:
   ```bash
   python hand_gesture_app.py
   ```

## Usage
- Ensure the webcam is connected and functioning.
- The GUI will allow you to adjust the detection and tracking confidence for better accuracy.
- Detected gestures will trigger color changes on the connected RGB LED via the ESP32.

## License
This project is licensed under the MIT License.
