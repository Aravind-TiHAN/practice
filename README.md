# Solio-S186

## Table of Contents
- [Introduction](#introduction)
- [Procedure to Run the Program](#procedure-to-run-the-program)
- [Hardware Configuration of Orin](#orin---computing-device)
- [Perception Models for Autonomous Vehicles](#1️⃣-perception-models-for-autonomous-vehicles)
- [Navigation File](#2️⃣-navigation-file)
- [TiHAN Testbed Route](#tihan-testbed-route)

<p align="center">
  <img src="https://cdn.wheel-size.com/automobile/body/suzuki-solio-bandit-2020-2022-1626347862.1116588.jpg" alt="Solio" width="48%" />
  <img src="/home/ninerishav/Suzuki_S186_Demo/pictures/IITHLogo_V.png" alt="IITH" width="30%" />
</p>

## Procedure to Run the Program

First, run the Perception file followed by the Navigation code.

```bash
sudo -S chmod 777 /dev/ttyUSB       # for giving the permission to the GNSS
roslaunch novatel_oem7_driver oem7_tty.launch oem7_tty_name:=/dev/ttyUSB0       # To do the roslaunch and start GNSS receiver
python P1-perception.py
python N1-navigation.py
```

## Orin - Computing Device
<img src="https://www.nvidia.com/content/dam/en-zz/Solutions/gtcf22/embedded-systems/jetson-orin/jetson-agx-orin-developer-kit-assets-2c50-p@2x.jpg" alt="Orin" width="600" height="400">

### Hardware Configuration of Orin

The NVIDIA Orin is a high-performance computing device designed for autonomous systems and AI applications. Follow these steps to configure the hardware for this project:

#### Power Supply
- Ensure that the Orin device is connected to a stable power supply with the appropriate voltage and current ratings as specified by NVIDIA.

#### Network Connection
- Connect the Orin device to the network via Ethernet to ensure it can communicate with other system components, such as the GNSS receiver and vehicle controller(MABX).

#### Peripheral Connections
- **GNSS Receiver:**
  - Connect the GNSS receiver to the Orin device via a serial port or USB port.

- **Camera:**
  - Connect the cameras (e.g., ZED camera) to the Orin device using USB-3

#### Display and Input Devices
- Connect a monitor to the Orin device using HDMI or DisplayPort.
- Connect a keyboard and mouse via USB for initial setup and configuration.

#### Storage : 64 GB
- Ensure that the Orin device has sufficient storage for the operating system, ROS, and other software dependencies. 

#### Memory : 32 GB 
#### Processor : ARMv8 Processor rec 1 x 12
#### Graphic Card : NVIDIA tegra Orin(nvgpu)/integrated
#### Cooling System
- Ensure that the Orin device is properly cooled using heatsinks or fans as recommended by NVIDIA to prevent overheating during operation.

```bash
sudo /usr/bin/jetson_clocks --fan
```


<img src="pictures/vlcsnap.png" alt="TestbedImage">


## [1️⃣Perception Models for Autonomous Vehicles](P1-perception.py)


### Overview

This project integrates multiple perception models to enhance the situational awareness and decision-making capabilities of autonomous vehicles. The system uses a ZED camera for capturing images and depth information, and YOLO models for detecting traffic lights, traffic signs, collision warnings, speed bumps, potholes, and pedestrian intentions.

To run this file:
```bash
python P1-perception.py
```
### System Components

#### 1. ROS Node Initialization

- **Node Name**: Perception_models
- **Publishers**:
  - `Traffic_light_type`
  - `Traffic_light_depth`
  - `Traffic_sign_type`
  - `Traffic_sign_depth`
  - `CW_type`
  - `CW_depth`
  - `CW_side_dist`
  - `SB_Pothole_type`
  - `SB_Pothole_depth`
  - `/intention`

#### 2. YOLO Models

The following YOLO models are loaded onto the GPU:

- `model1`: Traffic Light Detection
- `model2`: Traffic Sign Detection
- `model3`: Collision Warning Detection
- `model4`: Speed Bump and Pothole Detection
- `model5`: Pedestrian Intention Detection

#### 3. ZED Camera Configuration

- **Resolution**: HD720
- **FPS**: 30
- **Depth Mode**: NEURAL
- **Coordinate System**: RIGHT_HANDED_Z_UP_X_FWD
- **Depth Maximum Distance**: 40 meters

#### 4. Image and Point Cloud Retrieval

- Retrieve left image and point cloud data from the ZED camera.

#### 5. Inference Execution

- Perform inference using the YOLO models in separate threads for efficiency.

#### 6. Object Detection and Tracking

- **Traffic Lights**: Detect and publish the nearest traffic light and its distance.
- **Traffic Signs**: Detect and publish the nearest traffic sign and its distance.
- **Collision Warnings**: Detect and process objects for potential collision warnings.
- **Speed Bumps and Potholes**: Detect and publish the nearest speed bump or pothole and its distance.
- **Pedestrian Intention**: Track and determine the intention of detected pedestrians (crossing, intending to cross, not crossing).

### Key Functions

#### Model Inference

- **`model_inference(frame, model)`**: Executes model inference on the provided frame.
- **`collisionWarning_inference_specific(frame, model)`**: Executes model inference for specific classes related to collision warnings.

#### Object Processing

- **`nearest_traffic_light_OR_sign(lights, depth)`**: Determines the nearest traffic light or sign.
- **`process_collision_warning(CW_objects, distance_to_object, side_distance)`**: Processes collision warnings based on detected objects and their distances.
- **`nearest_speed_bump_or_pothole(SB_Pothole_objects, distance_to_object)`**: Determines the nearest speed bump or pothole.

#### Utility Functions

- **`xywh2abcd(xywh, im_shape)`**: Converts bounding box coordinates.
- **`detections_to_custom_box(detections, im0)`**: Converts detections to custom box format for ZED SDK.
- **`check_object_crossing(obj_data)`**: Checks if a detected object (pedestrian) is crossing the road.

### Visualization

- Display the results using OpenCV windows, showing bounding boxes and labels for detected objects.

### ROS Publishers

- Publish detected objects and their distances to the respective ROS topics for further processing by the autonomous vehicle's navigation system.

### Running the Code

1. Ensure the ZED camera is connected and its serial number is set correctly.
2. Install the necessary libraries and dependencies.
3. Run the script to start the perception models and publish the detections to the ROS topics.



## [2️⃣ Navigation File](N1-navigation.py)

### Autonomous Vehicle Navigation Using GNSS and Perception Data

This repository contains a Python script for navigating an autonomous vehicle using GNSS (Global Navigation Satellite System) data and perception data from various sensors. The system is designed to handle real-time navigation tasks, including collision warnings, traffic light and sign recognition, and pedestrian intention detection.

To run this file:
```bash
python N1-navigation.py
```
#### Features
- **GNSS Data Integration:** Uses GNSS data to determine vehicle position, velocity, and heading.
- **Perception Data Handling:** Processes collision warnings, traffic light and sign information, speed bumps, and pedestrian intentions.
- **Vehicle Control:** Sends control commands to the vehicle's controller via a UDP socket.
- **Logging:** Logs navigation and perception data for debugging and analysis.


#### Method Flow

Initialization
- ROS Node Setup
- Socket Setup
- Logging Setup

ROS Perception Subscribers
- Collision Warnings
- Traffic Lights and Signs
- Speed Bump & Pothole
- Pedestrian Intention

Callback Functions from GNSS
- Velocity Updates
- Heading Updates
- Latitude and Longitude Updates
- GNSS IMU Data

#### Navigation Variables and Their Functions

##### Speed and Turning Parameters

- **speed**: The base speed (in km/h) for the vehicle.
- **turning_factor**: The factor by which speed is reduced during turns.
- **STEER_GAIN**: Default gain for steering calculations. This value can change based on the bearing difference:
  - `STEER_GAIN = 300` when `abs(bearing_diff) < 1` (for very small bearing differences).
  - `STEER_GAIN = 1200` when `abs(bearing_diff) > 20` (for large bearing differences).

##### Waypoint Parameters

- **next_wp**: The number of waypoints ahead to consider when preparing for a turn.

##### Collision Warning (CW) Parameters

- **side_distance**: 1.1  
  The side distance (in meters) within which a collision warning is considered.
- **detecting_distance**: 35  
  The distance (in meters) within which collision warnings are detected.
- **stop_distance**: 10  
  The distance (in meters) within which the vehicle should stop for a collision warning.
- **caution_distance**: 20  
  The distance (in meters) within which the vehicle should slow down for a collision warning.

##### Traffic Light Parameters

- **threshold_distance**: 21  
  The distance (in meters) within which the vehicle should stop for a red traffic light.

##### Speed Bump Parameters

- **sb_threshold_distance**: 15  
  The distance (in meters) within which the vehicle should slow down for a speed bump.



## TiHAN Testbed Route

<img src="pictures/testbedMAP.jpg" alt="TestBed Map" height="750">

