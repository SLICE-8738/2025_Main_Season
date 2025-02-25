package frc.robot.subsystems;

import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.slicelibs.TalonFXPositionalSubsystem;
import frc.slicelibs.config.CTREConfigs;

public class Elevator extends TalonFXPositionalSubsystem {

    // private DigitalInput bottomLimitSwitch;
    //private DigitalInput topLimitSwitch;

    public Elevator(int[] ids, boolean[] inverted, double kP, double kI, double kD, double positionConversionFactor,
            double velocityConversionFactor) {
        super(ids, inverted, kP, kI, kD, GravityTypeValue.Elevator_Static, positionConversionFactor,
                velocityConversionFactor, Constants.CTRE_CONFIGS.elevatorFXConfig);
        setEncoderPosition(0);

        // this.bottomLimitSwitch = bottomLimitSwitch;
        // this.topLimitSwitch = topLimitSwitch;
    }

    public void moveTo(double height) {
        setPosition(height);
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
    }
}
