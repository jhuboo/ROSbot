# Introdution

## Structure of Robot

Robot Definition
> A robot is a machien that combines electro-mechanical systems, computing, sensing, and actuation; its capabilities can fall along a large spectrum of intelligence and autonomy.

The above definition implies the following cycle: sensing -> computation -> actuation -> effect with environment -> sensing


### Components

1. Mechanical System
	- Comprises of the physical components of the robot and determines how the robot interacts with the environment (actuation). This is abrasted through a model, generally in the form of an ODE, or state machine, or combination of both.

2. Perception
	- Covers sensors and algorithms the robot can use of get a sense of the state of its environment. Ie, use an IR sensor to know how far it is from a barrier, or use a cameraand analyze it to perform a certain task.

3. Control & Decision Making
	- Algorithms that command the robot to move according to its high-level objectives and by reacting to its environment. Ie, use LIDAR to know when it reaches a target, then pick something off a table.

4. Mapping & Localization
	- Methods to model and represent the state of the environment from the perspectiv of the robot. Using sensor data & computations, or ML algorithms, the robot can build a map of the different obstacles in the environment.

5. Planning
	- Methods and algorithms to go FROM environment repr together with high-level objectives TO low-level plan for control of robot (ie. trajectory).


### Time Scales

1. Fast Time Scale
	- Typically associated with low-level perception and control, which allows to quickly react to noise and disturbances, but cannot reliably achieve objs with long-time horizons. Ie, microcontrollers like Arduino used to compute fast time scale control.

2. Slow Time Scale
	- Typically associated to mapping and high-level planning. Algorithms running at this time scale are not fast enough to cope with low latency, to environmental changes, but allow to consider long-term objectives. Ie, Single Board Computers (SBCs), like the Pi are good for slow time scale operations since it has an onboard OS.


## Hardware Organization (Computation, Sensors, Actuators)

### Computation

1. Embedded Systems
	- Computers dedicated to specific purposes in an electro-mechanical system

2. Microcontrollers
	- No OS, strigent real-time, low-level. Good for very low-level control (ie. motor speed) and interfacing (ie. get dat from an analog sensor). Ie. Arduino

3. Single Board Computers (SBCs)
	- OS (typically Linux variant), Non-Strigent real-time, High-level. Ie. Raspberry PI


### Sensors

1. Monocular & Stereo Cameras
	- *Monocular*: camera has only one lens and image sensor. Only 2D image capable.
	- *Stereo*: camera has 2 or more lenses with separate image sensor for each lens. This allows the camera to capture 3D images by correlating the two images and triangulating the points in space.

2.  Depth Cameras
	- *Structured Light Cameras*: structured light from IR emitter is emitted onto the object. Camera detects IR that bounces back. Camera takes picture of sturctured light distorted by the geometry of the object. By correlating the distorted pattern with the one orignally projected, it is possible to recover the depth information via triagulation method. The larger the distance, the more accurate the measurements, but self-occlusion become more pronounced.
	- *Time of Flight Cameras*: spatially uniform but time-modulated light is projected on the scene. The time difference between emission and reception at the sensor is proportional ot the distance.

3. Ultrasonic Sensors
	- Sensor measrues distance by recording time it takes for sound wave to return. However, if object is soft, porous or oriented in a certain way, it may be undetectable.

4. LIDAR
	- Combination of scanning laser & monocular camera. Functions similar to depth camera but instead of emitting flash of light rays, it scans the environment with a single light ray. Measures the reflected light pulses, and return time with a light sensor. Can be 2D or 3D.

5. GPS
	- Constellation of 35 satellites in LEO orbit Earth 2x per day. Every spot on Earth is in sight of four satellites at a time. Each satellite has an atomic clock. This system works by communicating with GPS receivers on land. The satellite sends a signal to the receiver tracking the exact time the signal was sent and the exact time it was received. The receiver is sent a signal from at least 3 satellites and uses triangulation to know their exact position in the sky (by calculating the time it takes to get signals from 3 GPS satellites, it can determine its position on Earth!). The 4th satellite is used to keep time because the GPS receiver is not armed with an atomic clock. The receiver will know its East, North, and altitude positions.

6. IMU (Inertial Measurement Units)
	- Device that measures an object's specific force, magnetic field, and angular rate with accelerometers, magnetometers, and gyroscopes. IMU works in the following way: An accelerator works by measuring changes in acceleration an object undergoes by first measuing the change in voltage. Say there is a spring and mass model. A known mass is contained in the box, and suspended by springs on all sides. When the box, and mass have a force applied to them, and move, the springs deform. Using circuitry, the deformation can be converted into a voltage. This voltage change is sent through an AC to DC converter, and turned into a number presenting acceleration. A comman way voltage into acceleration is incorporating capacitors into the mass-spring model. With a capacitor on th emass and on the walls of the box, any distance the mass moves will change the voltage going through the capacitors on the wall. Therefore, the change in voltage represents a change in accelerative forces, and be integrated into a distance.


### Actuators
> Typically represented by motors. Other options include piezoelectric actuators (for high-frequency operation such as RoboBee) and soft actuators

1. Brushed DC Motors
2. Brushless DC Motors

