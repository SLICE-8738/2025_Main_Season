package frc.robot.subsystems;

import frc.slicelibs.PositionalSubsystem;

public class Elevator extends PositionalSubsystem{
    private static final double LEVEL_ONE = 0;
    private static final double LEVEL_TWO = 1;
    private static final double LEVEL_THREE = 2;
    private static final double LEVEL_FOUR = 4;

    private int[] ids;
    private boolean[] inverted;
    private double kP;
    private double kI;
    private double kD;
    private double positionConversionFactor;
    private double velocityConversionFactor;
    
    public Elevator(int[] ids, boolean[] inverted, double kP, double kI, double kD, double positionConversionFactor, double velocityConversionFactor){
        super(ids, inverted, kP, kI, kD, positionConversionFactor, velocityConversionFactor);
        setEncoderPosition(0);

        ids = this.ids;
        inverted = this.inverted;
        kP = this.kP;
        kI = this.kI;
        kD = this.kD;
        positionConversionFactor = this.positionConversionFactor;
        velocityConversionFactor = this.velocityConversionFactor;
    }
    
    public void moveToLevelOne(){
        setPosition(LEVEL_ONE * positionConversionFactor);
    }

    public void moveToLevelTwo(){
        setPosition(LEVEL_TWO * positionConversionFactor);
    }

    public void moveToLevelThree(){
        setPosition(LEVEL_THREE * positionConversionFactor);
    }

    public void moveToLevelFour(){
        setPosition(LEVEL_FOUR * positionConversionFactor);
    }
}
