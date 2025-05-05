# The End Effector Subsystem

## About
The end effector is the subsystem that is used to interact with the Coral and Algea game elements.
   ### End Effector Functions
        The end effector is able to perform and fulfill many functions
        * Index Coral from the Human Player Station Intake
        * Score Coral onto the Reef
        * Remove Algae from the Reef
        * Carry Algae (kind of)
   ### Hardware /Components
   The end effector 

---
## The Code (EndEffector.java)
The EndEffector.java class is a subclass of the TalonFXPositionalSubsystem.java class
For information and references on how the TalonFXPosititionalSubsystem superclass works, view its documentation page
   ### Class and Instance Variables
   * `private DutyCycleEncoder encoder` :
    The code object representing the interface that allows the code to interact with the encoder on the motor
    which helps keep track of the relative location of the end effector
   * `private static CANrange frontSensor` :
    the code object representing one of the many sensors on the End Effector 
    which are used to detect whether or not there is a coral within the end effector
        - Same for `middleSensor`, `topBackSensor`, and `bottomBackSensor`
   * `private TalonFX placementMotor` :
    The code object representing the motor used to manipulate the Coral game piece
        - The motor that controls the rotation of the End Effector is passed through as an argument of the SuperClass as `new int[] { Constants.kEndEffector.ROTATION_MOTOR_ID }`
   ### The Constructor
   ### Methods
   

---
## The Design Process