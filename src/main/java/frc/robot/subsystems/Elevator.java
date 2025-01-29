package frc.robot.subsystems;

import frc.slicelibs.PositionalSubsystem;

public class Elevator extends PositionalSubsystem{

    private int[] ids;
    private double positionConversionFactor;
    
    public Elevator(int[] ids, boolean[] inverted, double kP, double kI, double kD, double positionConversionFactor, double velocityConversionFactor){
        super(ids, inverted, kP, kI, kD, positionConversionFactor, velocityConversionFactor);
        setEncoderPosition(0);

        this.ids = ids;
        this.positionConversionFactor = positionConversionFactor;
    }
    
    public void moveTo(double height){
        setPosition(height * positionConversionFactor);
    }
}
