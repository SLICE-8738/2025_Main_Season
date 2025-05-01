This season we used an elevator that could extend up to [X] feet. It has two motors that move in opposite directions to move up and down.

## Mechanism Goals

- Go up and down as fast as possible without breaking anything
- Have setpoints at key heights that it can reach reliably and accurately(within half a centimeter)

## TalonFXPositionalSubsystem
We use the `TalonFXPositionalSubsystem` because the elevator has two motors that run at the same time at the same speed as each other.

- Parts of the `TalonFXPositionalSubsystem` `super()` method
    
    - `int[] ids`: This is an array of the motor ids that need to move in tandem(note that the order the motors are in is the order any TalonFXPositionalSubsystem methods will output). We have two motors in this subsystem so we make an array in line with the motor ids taken from the Constants file/

    - `boolean[] inverted`: This is an array of booleans to determine which motors are inverted. This needs to be the same size as `ids` because this is setting a config on each motor. We inverted the left motor so the array is `{true, false}`

    - `double kP`: This is the P value the motor controller uses when it is told to go to a position. P is essentially the speed that the motors initally use to try to get to the position. The value we used was aquired from slowly raising and testing the `kP` and seeing if it could reach its target position(we started this testing with a value of 0.25)

    - `double kI`: This value effects how much the motor speeds up as time goes on without reaching its target. This value was achived from slowly raising it until we felt it was fast enough for our liking(initially set to 0 until `kP` was tuned).

    - `double kD`: This value effects how much the speed "dampens" as it gets closer to its target. This value too was gotten through raising it until it felt right(initially set to 0 until `kP` and `kI` were tuned)

    - `double kG`: This value effects the automatic anti-gravity. Again vibes based tuning set it until it felt right.

    - `double sensorToMechRatio`: [GET GABE TO DO THIS I FORGOT WHAT THIS WAS EXACTLY]

    - `GravityType gravityType`: The type of gravity the mechanism uses. This is an elevator so we chose `Elevator_Static`

    - `double positionConvertionFactor`: How much distance per motor rotation. We used meters so we calculated in terms of meters per rotation

    - `double velocityConversionFactor`: How many rotations per second is equivalent to speed per second. Calculated based on meters.

    - `TalonFXConfiguration motorConfigs`: Any other configurations you desire on your subsystem. This elevator uses `elevatorFXConfigs` from `CTRE_CONFIGS`

## Constants
Every subsystem has constants that they use. These are stored in the `Constants.java` file.

- `int LEFT_MOTOR_ID` and `int RIGHT_MOTOR_ID`: These are the ids for the left and right motor of the elevator. They can be found through the Phoenix Tuner X application.

- `double POSITION_CONVERSION_FACTOR`: These are for the motor config values mentioned above. It was calculated as (pitch circumference) divided by (gear ratio) 

## Methods

- `public void moveTo(double height)`: This method, in hindsight, serves no purpose. Just call the `setPosition(double position)` method directly from `TalonFXPositionalSubsystem`
