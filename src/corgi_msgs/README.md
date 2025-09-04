# Corgi Messages (`corgi_msgs`)

This package defines all custom ROS messages (`.msg`) and services (`.srv`) used throughout the Corgi robot workspace. These custom data structures are essential for ensuring consistent and clear communication between the various nodes, such as the controllers, state estimators, simulation, and hardware interfaces.

## Package Dependencies

This package is a core dependency for nearly all other `corgi_*` packages in the workspace. It depends on the following standard ROS packages:
* `std_msgs`
* `sensor_msgs`
* `geometry_msgs`

## Message and Service Definitions

Below is a complete list of all messages and services defined in this package. Many messages have a "Stamped" version, which wraps the base message with a timestamped header for synchronization purposes.

### Messages (`.msg`)

* #### Core & Utility Messages
    * **`Headers.msg`**: A custom header structure containing a sequence number, timestamp, and frame ID.
    * **`TriggerStamped.msg`**: A generic, timestamped trigger message used to enable/disable processes and specify output filenames.
    * **`SensorEnableStamped.msg`**: A timestamped command to enable or disable specific sensors like the camera, lidar, or IMU.

* #### Motor & Power Messages
    * **`MotorState.msg`**: Contains the measured state (angles `theta`/`beta`, velocities, torques) of a single leg module's two motors.
    * **`MotorStateStamped.msg`**: A timestamped message that aggregates the `MotorState` from all four leg modules.
    * **`MotorCmd.msg`**: Defines the desired command (angles `theta`/`beta`, PID gains, feed-forward torques) for a single leg module's two motors.
    * **`MotorCmdStamped.msg`**: A timestamped command that bundles `MotorCmd` for all four leg modules.
    * **`PowerStateStamped.msg`**: A comprehensive, timestamped report of the system's power status, including individual motor voltage and current.
    * **`PowerCmdStamped.msg`**: A timestamped command to control the system's power rails and trigger steering calibration.

* #### Whole-Body & Leg Control Messages
    * **`ForceState.msg`**: Reports the measured X and Y forces for a single leg.
    * **`ForceStateStamped.msg`**: A timestamped message aggregating the `ForceState` from all four legs.
    * **`ImpedanceCmd.msg`**: Defines impedance control parameters (target angles, forces, stiffness, damping) for a single leg.
    * **`ImpedanceCmdStamped.msg`**: A timestamped command that bundles `ImpedanceCmd` for all four legs.
    * **`ContactState.msg`**: A boolean and a confidence score indicating the contact state for a single foot.
    * **`ContactStateStamped.msg`**: A timestamped message aggregating the `ContactState` from all four feet.

* #### Wheel & Steering Messages
    * **`WheelCmd.msg`**: A command message for the drive wheels, specifying speed, direction, and stop commands.
    * **`SteeringStateStamped.msg`**: A timestamped message reporting the current angle and status of the steering motors.
    * **`SteeringCmdStamped.msg`**: A timestamped command for the desired angle and voltage of the steering motors.

* #### State Machine & Perception Messages
    * **`FsmStateStamped.msg`**: A timestamped message reporting the current state of the Finite State Machine (FSM), including the next mode and status flags.
    * **`FsmCmdStamped.msg`**: A timestamped command to trigger a state transition in the FSM.
    * **`StairPlanes.msg`**: Contains information about detected horizontal and vertical stair plane geometry from a perception node.

* #### Simulation Messages
    * **`SimDataStamped.msg`**: A timestamped message for sending comprehensive data from the simulation environment, including position, orientation, and contact points.
    * **`SimContactPoint.msg`**: Reports detailed contact point information within the simulation.

### Services (`.srv`)

* **`imu.srv`**: A service to send a command (`mode`) to the IMU and receive a status reply (`mode`), typically used to trigger actions like calibration or resetting its state.