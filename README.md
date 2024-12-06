# ESP32-motion-tracking-system

## Abstract
This project makes use of an ESP32 Camera microcontroller, combined with a stepper motor, to create a real-time tracking system. 
The main idea behind it was to create a simple system that could be able to track objects moving within the range of the ESP32's camera module.
By using Proportional–integral–derivative controller the system ensures an improved precision whilst targeting the entity found in motion.
It is also possible, as explained below, enable a real-time WebServer to observe the camera's stream.
## Technical Overview

### Components:
- **ESP32 Camera Microcontroller**: Provides the computational power to process the data from the integrated camera module and control the stepper motor.
- **28BYJ-48 Stepper Motor**: Responsible for moving the camera and targeting system to follow the detected entity.
- **ULN2003 Motor Driver**: Drives the stepper motor based on the input signals from the microcontroller.
- **Power Supply**: External 9V power supply for the stepper motor and USB or 3.3V for the microcontroller.
- **Potentiometer**: Potentiometer used to home the stepper if option is enabled.
- **Laser Module**: The laser module (AKA BFG 11K or Nuclear Laser Beam) is used to shoot at the entity identified by the camera as motion.

### Functionality:
 
When motion is detected inside the camera's FOV, the stepper motor adjusts its position to follow the movement of the object. As soon as the detected motion results "still" and the motor has reached the "final" position, a laser beam is shot in the moving entitiy's direction. The entire process is controlled by the ESP32 microcontroller, which processes erything in real-time.

### Key Features:
- **Real-time Motion Detection**: Utilizes the ESP32's camera module to capture frames and detect motion through pixel comparison.
- **Automated Targeting**: The stepper motor adjusts its position automatically to follow moving entities inside the camera's Field of View.
- **Customizable Parameters**: Thresholds for motion detection, speed of the stepper motor, and frame resolution can be fine-tuned for specific use cases.

- **(BETA) Stepper Motor Homing**: by using a potentiometer it's possible to set the stepper home position each time the system is powered up. This  could  
- **Real-Time WebServer**: if enabled it is possible to connect to x.x.x.x/stream and observe the real time stream of the camera.

### How It Works:
1. **Frame Capture**: The camera module captures video frames at a predefined resolution (defeault is 320x240).
2. **Motion Detection**: The ESP32 processes the frames to detect changes by comparing pixel differences between consecutive frames.
3. **Motor Control**: If motion is detected, the microcontroller calculates the horizontal direction of movement and sends signals to the stepper motor driver.
4. **Tracking**: The stepper motor moves in response to the detected movement, adjusting the direction of the mounted BFG 11K.
5. **Shooting**: The BFG 11K shoots its laser beam upon the surface of the identified entity

#### Motion Detection Logic
The main idea behind the motion detection logic is the analysis of pixel differences in the field of view of the camera.
When acquiring a frame, the ESP32 divides it in some regions (the value is defined by the `REGIONS` variable) and calculates the total amount of pixels in that region saving it inside an array called current_region_sums. As soon as another frame is captured, the current value of `current_region` is transfered in `previous_region_sums` array.

In order to detect motion, a comparison between `current_region` and `previous_region_sums` has to be done. The region with most difference is the region that most likely has motion in it.

#### Calculating the stepper motor's destination
The calculation of the stepper motor's destination is based on aligning its movement with the detected region of motion in the camera's FOV. The FOV represents the angular coverage of the camera, which is divided into `REGIONS` along the horizontal axis. Each region corresponds to a specific segment of the image width. When motion is detected in a region, the system calculates its offset from the center (mid-region) and converts this into a pixel offset. This pixel offset is mapped to an angular offset by determining the proportion of the offset relative to the total resolution width of the camera. This proportion is then multiplied by the camera's horizontal Field of View (FOV), effectively translating the linear pixel movement into an angular measurement. The angular offset represents how far the motor needs to turn to align the laser or device with the center of the detected region. Finally, this angular offset is converted into motor steps by multiplying it by the motor's steps-per-degree value, ensuring the motor moves precisely the required distance to center on the region of interest.
### Configuration
For the configuration, you should consider 1 as YES and 0 as NO
```
#define SERIAL                          --> Serial Output
#define ENABLE_WEB_SERVER               --> Enable Web Server
#define CAMERA_MODEL_AI_THINKER         --> Camera Model
#define FIX_HOMING                      --> Homing Setup feature

#define FRAME_SIZE                      --> Frame Size
#define RES_WIDTH                       --> Frame Width
#define RES_HEIGHT                      --> Frame Height

#define STEPS_PER_DEGREE                --> How many steps per degree
#define STEPS_PER_REVOLUTION            --> How many steps per revolution
#define STEPPER_RPM                     --> Stepper motor's RPM
#define POTENZIOMETER_THRESHOLD         --> Threshold for noise filtering

#define REGIONS                         --> FOV's REGIONS
#define MID_REGION                      floor(REGIONS/2)
#define CHANGE_THRESHOLD                --> Motion detection Threshold
#define HORIZONTAL_PIXELS_PER_REGION    floor(RES_WIDTH/REGIONS)      
#define FOV_HORIZONTAL                  --> Camera module's FOV
```
### Challenges:
- **Power Management**: The system requires separate power sources for the microcontroller and stepper motor, ensuring stability.
- **Frame Processing**: Efficiently processing frames in real-time while avoiding delays or missed movements.
- **Stepper Motor Precision**: Fine-tuning the stepper motor for smooth and accurate targeting based on the detected motion.

### Future Improvements:
- **Object Recognition**: Integrating object recognition to follow specific types of objects.
- **Wireless Control**: Adding remote control and monitoring functionality through Wi-Fi or Bluetooth.
- **Higher Precision**: Enhancing the accuracy of motion detection and motor movement for applications requiring fine adjustments.
- **NERF Gun**: Adding support for a nerf gun would definitely be fun.

