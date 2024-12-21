# Ninjineers 2025 Base Robot Code

FRC Robot Code has gotten significantly more advanced in recent years with brushless motors, external CAN sensors, vision, and more complicated odometry. Ninjineers has realized that these advancements are not possible over the course of a single season so we created this base code to allow teams to focus on the emergent behavior of their robot instead of getting bogged down in the weeds of motor control. In developing this template we tried to create sensible defaults for Falcons, Krakens, and Spark Maxs for motor control, and integrated Mechanical Advantage's Drivetrain and Vision projects (with modification to enable more modular configuration). These defaults work for us but you are free to create your own [IO layers](https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces) or bespoke Subsystems. We would love it if you create a pull request to share your code with other teams.

## Getting Started

1. Create a fork of this repo, this will allow you to pull the latest changes in this repo directly from the Github UI.
2. Pull your fork
3. Split up your robot into different subsystems (Check out the [Subsystems](#subsystems) section)


## Subsystems

We split our robot into many small subsystems to allow for heavy reuse of subsystems (this is why we have only two motor control subsystems).

* Flywheel

  Flywheel is the more basic of the two subsystems and allows for controlling the velocity or voltage of a single motor or a group of motors. The biggest constraint with a Flywheel subsystem is it can only have one velocity setpoint or velocity measurement. This means that every motor will be spinning together (probably in the same gearbox).

* PositionJoint

  PositionJoints are more complicated as they can represent a Pivot or Elevator. A PositionJoint supports Position and Voltage control modes. Just like the Flywheel a single PositionJoint can also only have one position setpoint and position output but can have multiple motors.

### Splitting a Subsystem

Many of what we want to think of as a single robot Subsystems will need to be represented as multiple Subsystems in code. For example: a shooter from Crescendo might have a set of left wheels and a set of right wheels that spin at different velocities to create spin. This should be represented as two Flywheels (one for the right and one for left). The differential velocity is a more complicated emergent behavior that should be controlled at the command level not the subsystem level.

## Flywheel Constants

Flywheels have PIDF gains and motor configs:
* Gains

  Feedback:
  kP: (volts / rotation / second)
  kI: (volts / rotation)
  kD: (volts / rotation / second^2)

  Feedforward:
  kS: (volts)
  kV: (volts / rotation / second)
  kA: (volts / rotation / second^2) (Not recommended to be non 0) (Not used in SparkMax control)

* Motor Configs
  canIds: List of canIDs for motors in group. First canID will be master
  reversed: Reverse config for each motor: first boolean will reverse entire group, next booleans are all relative to master motor
  gearRatio: ratio of mechanism to motor
  canBus: For TalonFX can be name of CANivore or "rio", unused on SparkMax

## PositionJoint Constants

* Gains
  Very similar to Flywheel with minor differences noted here
  kG: Feedforward to compensate for effects of gravity
  kTolerance: Position tolerance for joint to be considered at setpoint

* HardwareConfig
  gearRatio: ratio of mechanism (radians or meters) to motor (rotations)
  gravity: GravityType.Constant for an elevator, and GravityType.COSINE for pivots. (This means that pivots must have a 0 position horizontal)
