package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.slicelibs.PositionalSubsystem;

public class Elevator extends PositionalSubsystem{

    private DigitalInput bottomLimitSwitch;
    private DigitalInput topLimitSwitch;
    
    public Elevator(int[] ids, boolean[] inverted, double kP, double kI, double kD, double positionConversionFactor, double velocityConversionFactor, DigitalInput bottomLimitSwitch, DigitalInput topLimitSwitch){
        super(ids, inverted, kP, kI, kD, positionConversionFactor, velocityConversionFactor);
        setEncoderPosition(0);

        this.bottomLimitSwitch = bottomLimitSwitch;
        this.topLimitSwitch = topLimitSwitch;
    }
    
    public void moveTo(double height){
        setPosition(height);
    }

    public boolean isAtBottom(){
        return bottomLimitSwitch.get();
    }

    public boolean isAtTop(){
        return topLimitSwitch.get();
    }
}
