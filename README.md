Codebase used on the 8889A Worlds Robot during the 2025-2026 VEX season "Push Back"

This project was originally intended to be a library similar to other VEX projects such as JAR-Template or LemLib, but was never finished.

The algorithms used in the codebase took over a thousand hours of testing and debugging over High Stakes and Push Back. Every little tip such as “the optical sensors are inconsistent because they update at 100ms intervals by default” took hours of debugging. 

Tips for VEX programming:
Consistency is more important than speed, don’t reduce consistency trying to squeeze half a second out of a rush.
Consider programming/tracking when designing the robot such as distance, optical, and line tracker sensor placement.
No code will ever be one hundred percent consistent. When the code misses once in a match, don’t rush to change things before the next match. 
Don’t start your code while carrying or moving the robot or the IMU will drift.
Plug the controller in before turning the robot on to prevent disconnects.
This may be outdated for future VEX games because of the addition of apriltags. However, prior to this, these were the minimum setups for good localization (through experience):
Traction wheel on drivetrain, no tracking wheels, no distance sensors, relative PID
Traction wheel on drivetrain, vertical tracking wheel, no distance sensors, odometry
No traction wheel, horizontal + vertical tracking wheel, no distance sensors, odometry
Traction wheel, no tracking wheels, distance sensors, odometry
No traction wheel, no tracking wheels, 6+ distance sensors, odometry

The codebase supports many unique features and algorithms not commonly used by other teams. Below is a summary of the codebase:

miku::Controller (miku/devices/controller): 
The VEX controller is difficult to program. Only one line update is permitted at a time, including controller rumbles. 
The miku::Controller wrapper automatically handles 3-line display updates of lambda functions, making the controller display easy to use for debugging.
The default update is 100ms per line, meaning it takes 300ms to update 3 lines. The interval is adjustable, but should not be lowered due to potential for crashes.
It also handles controller rumbles, which are given higher priority than line updates. An unlimited string pattern can be held by the controller.
The controller has a feature to toggle continuous rumbling at a set interval. This can be useful to give the driver information about a state, such as whether a hidden piston is activated or not.
Lastly, the controller can handle a queue of timed alerts, that show on the screen and rumble the controller. These alerts can be used in debugging or to warn about a robot issue (port unplugged, motor overheat, etc.)

miku::Distance (miku/devices/distance):
The main purpose of the miku::Distance wrapper is to update data for the Monte Carlo Algorithm. 
The miku::Distance class holds a numeric ID(for logging) and a pose relative to the center of the robot(x, y, theta). 
The numeric ID is necessary for logging distance sensor data on the SD card, in order to identify the origin of each data packet.
The offset pose of each distance sensor is (x, y, theta) where +y is along the robot heading and +x is to the right of the robot. Cosine and sine are precalculated to save computation time.
miku::Distance::get_reading() checks to see if a sensor returns valid data. For this, it checks if the reading is within its max range(0-2000mm). If the sensor doesn't get a valid reading, it will output 10000mm.
It also checks if enough light is reflected to the sensor using get_object_size().

miku::Imu (miku/devices/imu.hpp):
The miku::Imu wrapper has a calibrate() function similar to LemLib that tries to initialize the inertial sensor multiple times in the event of failure. An alert is sent to the controller if the set number of calibration attempts all fail.

miku::Intake (miku/devices/intake):
The miku::Intake class was supposed to be a generalized class that could hold any robot intake with any number of motors. However, I got lazy and it only works for the specific robot the team used.
The miku::Intake class has an anti-jam algorithm, a system to detect blocks being held in a certain stage of the intake, and an intake movement queueing system.
On the worlds robot, the top-stage rollers needed to stop when a block was ingested. However, there was no space to fit an optical or distance sensor to detect the block. If the motor detected high torque for an extended period of time, it would stop spinning until a non-loading command was given.
The queueing system allows for one or multiple motors to spin at a certain RPM or voltage for a set period of time. This was extremely useful for situations such as scoring slowly on the middle goals in the Skills Challenge, as the intake could score in timed stages that got progressively slower.
The anti-jam system detects if one or multiple motors is spinning at high voltage but low velocity for an extended period of time. If so, all motors in the intake spin backwards for a short period of time. Then, there is a cooldown until the next anti-jam can be triggered, since the motors need to reaccelerate after having been spun backwards at high speed.

miku::Motor (miku/devices/motor):
The miku::AbstractMotor class is built off the pros::Motor class and adds better velocity estimation through get_filtered_velocity(). The filtering process is derived from SyLib, layering simple moving average, median, and an exponential moving average using maximum acceleration for alpha.
The miku::MotorController class adds better velocity control for the motors using PIDF control. It overwrites the default motor PID control and uses a custom feedforward lookup table, PID controller, and velocity filter for move_velocity().
The miku::Motor and miku::MotorGroup classes are both built off the miku::MotorController class, allowing for velocity filtering and control in either one motor or multiple motors treated as one. 

miku::Optical (miku/devices/optical.hpp)
The default optical class causes a lot of confusion in handling hue values, using the proximity feature to prevent seeing outside of the field, turning on the LED, and most importantly the default refresh rate of 100ms needing to be reduced in order to have consistent results(such as High Stakes colorsort).
The miku::Optical class is simply a convenience wrapper that handles all of these issues. miku::Optical::initialize() sets the refresh rate and turns on the LED. miku::Optical::get_color() handles hue values and checks if the object is within proximity.

miku::Pneumatic (miku/devices/pneumatics.hpp)
All the miku::Pneumatic wrapper does is hold an internal state of whether or not the pneumatic is activated so you don’t have to write state = !state on every single piston toggle. In the driver control loop, you can just write pneumatic.toggle().

The codebase uses the fmt library for convenience, and the pcg32.h library for random number generation in the Monte Carlo Localization algorithm. Additionally, it has an updated LVGL 9.3 version of gif-pros written by Joshua Liu(15797A) for gif display on the brain.

miku::Drive (miku/system/drive):
The miku::Drive class handles autonomous and driver control of a six-motor drivebase.

Data logging (miku/system/log):
The codebase’s logging system allows data to be converted into binary and written to the SD card in the brain. The user first defines an ordered template containing a vector of patterns of data types. The patterns may be looped. For example, the user can log the current time (UINT32), then the robot position/heading and the derivative ((FLOAT, FLOAT, FLOAT), REPEAT 2). By default, the filename is “/usd/log.bin” but it can be changed in the open_logs() parameter.

Monte Carlo Localization (miku/system/mcl.cpp):

// README unfinished
// - distance reset algorithm
// - particle updating
// - speed optimization
// - imu drift correction
// - torque based movement cancel
// - polar move to pose
// - motion queueing system
// - auto selector and imu check
// - trapezoidal integration in PID
// - derivative cancel while settling

Written by Andrew Pan
Dedicated to Abinav Kumar 
