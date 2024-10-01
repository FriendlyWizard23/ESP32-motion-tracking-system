# ESP32-motion-tracking-system

## Abstract
This project makes use of an ESP32 Camera microcontroller, combined with a stepper motor, to create a real-time tracking system. 
The main idea behind it was to create a simple system that could be able to track objects moving within the range of the ESP32's camera module.
By using Proportional–integral–derivative controller the system ensures an improved precision whilst targeting the entity found in motion.
It is also possible, as explained below, enable a real-time WebServer to observe the camera's stream.
## Technical Overview

### Components:
- **ESP32 Camera Microcontroller**: Provides the computational power to process the data from the integrated camera module and control the stepper motor.
- **28BYJ-48 Stepper Motor**: Responsible for moving the camera and targeting system to follow the detected object.
- **ULN2003 Motor Driver**: Drives the stepper motor based on the input signals from the microcontroller.
- **Power Supply**: External 9V power supply for the stepper motor and USB or 3.3V for the microcontroller.

### Functionality:
    The Precision Tracker system is capable of detecting motion by comparing consecutive video frames captured by the camera module. 
    When motion is detected, the stepper motor adjusts its position to follow the movement of the object. The entire process is controlled by the ESP32 microcontroller, which processes real-time data.

### Key Features:
- **Real-time Motion Detection**: Utilizes a camera module to capture frames and detect motion through pixel comparison.
- **Automated Targeting**: The stepper motor adjusts its position automatically to follow moving objects.
- **Customizable Parameters**: Thresholds for motion detection, speed of the stepper motor, and frame resolution can be fine-tuned for specific use cases.
- **Expandable Architecture**: The system can be expanded with additional sensors or motors for more complex applications.
- **Real-Time WebServer**: if enabled it is possible to connect to x.x.x.x/stream and observe the real time stream of the camera.

### How It Works:
1. **Frame Capture**: The camera module captures video frames at a predefined resolution.
2. **Motion Detection**: The microcontroller processes the frames to detect changes by comparing pixel values between consecutive frames.
3. **Motor Control**: If motion is detected, the microcontroller calculates the direction and magnitude of movement and sends signals to the stepper motor driver.
4. **Tracking**: The stepper motor moves in response to the detected movement, adjusting the camera or targeting system.

### Challenges:
- **Power Management**: The system requires separate power sources for the microcontroller and stepper motor, ensuring stability.
- **Frame Processing**: Efficiently processing frames in real-time while avoiding delays or missed movements.
- **Stepper Motor Precision**: Fine-tuning the stepper motor for smooth and accurate targeting based on the detected motion.

### Future Improvements:
- **Object Recognition**: Integrating object recognition to follow specific types of objects.
- **Wireless Control**: Adding remote control and monitoring functionality through Wi-Fi or Bluetooth.
- **Higher Precision**: Enhancing the accuracy of motion detection and motor movement for applications requiring fine adjustments.

