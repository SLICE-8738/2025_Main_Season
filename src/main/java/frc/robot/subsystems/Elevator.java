package frc.robot.subsystems;

import com.ctre.phoenix6.signals.GravityTypeValue;

//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.ElevatorPositionSelector;
import frc.slicelibs.TalonFXPositionalSubsystem;
//import frc.slicelibs.config.CTREConfigs;

public class Elevator extends TalonFXPositionalSubsystem {

    // private DigitalInput bottomLimitSwitch;
    //private DigitalInput topLimitSwitch;

    public Elevator() {
        super(
            new int[] {Constants.kElevator.LEFT_MOTOR_ID, Constants.kElevator.RIGHT_MOTOR_ID},
            new boolean[] {true, false}, 
            Constants.kElevator.KP, 
            Constants.kElevator.KI, 
            Constants.kElevator.KD,
            GravityTypeValue.Elevator_Static,
            Constants.kElevator.POSITION_CONVERSION_FACTOR,
            Constants.kElevator.VELOCITY_CONVERSION_FACTOR, 
            Constants.CTRE_CONFIGS.elevatorFXConfig);
            setEncoderPosition(0);

        // this.bottomLimitSwitch = bottomLimitSwitch;
        // this.topLimitSwitch = topLimitSwitch;
    }

    public void moveTo(double height) {
        setPosition(height, 0);
    }

    // public boolean isAtBottom() {
    //     return bottomLimitSwitch.get();
    // }

    // public boolean isAtTop() {
    //     return topLimitSwitch.get();
    // }

    public void periodic() {
        double[] positions = this.getPosition();
        SmartDashboard.putNumber("Elevator Height", (positions[0] + positions[1]) / 2.0);
        SmartDashboard.putNumber("Target Height", ElevatorPositionSelector.getSelectedPosition().height);
    }
}
