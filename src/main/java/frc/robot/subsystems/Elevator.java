package frc.robot.subsystems;

import com.ctre.phoenix6.signals.GravityTypeValue;

//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.kElevator.Level;
import frc.robot.Constants.kElevator.LevelType;
import frc.robot.ElevatorPositionSelector;
import frc.robot.commands.EndEffector.MotorIntakeAlgae;
import frc.slicelibs.TalonFXPositionalSubsystem;
//import frc.slicelibs.config.CTREConfigs;

public class Elevator extends TalonFXPositionalSubsystem {

    // private DigitalInput bottomLimitSwitch;
    // private DigitalInput topLimitSwitch;
    private static Level m_coralLevel = Level.LEVEL1;
    private static Level m_algaeLevel = Level.ALGAE1;
    private static Level m_sourceLevel;
    private static LevelType m_levelType;


    public Elevator() {
        super(
                new int[] { Constants.kElevator.LEFT_MOTOR_ID, Constants.kElevator.RIGHT_MOTOR_ID },
                new boolean[] { true, false },
                Constants.kElevator.KP,
                Constants.kElevator.KI,
                Constants.kElevator.KD,
                Constants.kElevator.KG,
                1,
                GravityTypeValue.Elevator_Static,
                Constants.kElevator.POSITION_CONVERSION_FACTOR,
                Constants.kElevator.VELOCITY_CONVERSION_FACTOR,
                Constants.CTRE_CONFIGS.elevatorFXConfig);
        setEncoderPosition(0);

        // this.bottomLimitSwitch = bottomLimitSwitch;
        // this.topLimitSwitch = topLimitSwitch;
    }

    public void moveTo(double height) {
        setPosition(height);
    }

    public void maintainPosition() {
        if (Math.abs(ElevatorPositionSelector.getSelectedPosition().height - getPositions()[0]) > 0.01) {
            setPosition(getPositions()[0]);
        }

    }

    public static Level getCoralLevel(){
        return m_coralLevel;
      }
    
      public static Level getAlgaeLevel(){
        return m_algaeLevel;
      }
    
      public static Level getSourceLevel(){
        return m_sourceLevel;
      }
    
      public static LevelType getLevelType(){
        return m_levelType;
      }
    
      public static void setCoralLevel(Level angle){
        m_coralLevel = angle;
      }
    
      public static void setAlgaeLevel(Level angle){
        m_algaeLevel = angle;
      }
    
      public static void setSourceLevel(Level angle){
        m_sourceLevel = angle;
      }
    
      public static void setLevelType(LevelType levelType){
        m_levelType = levelType;
      }

    // public boolean isAtBottom() {
    // return bottomLimitSwitch.get();
    // }

    // public boolean isAtTop() {
    // return topLimitSwitch.get();
    // }

    public void periodic() {
        double[] positions = this.getPositions();
        SmartDashboard.putNumber("Elevator Height", (positions[0] + positions[1]) / 2.0);
        SmartDashboard.putNumber("Target Height", ElevatorPositionSelector.getSelectedPosition().height);
        SmartDashboard.putNumber("Elevator Current", getStatorCurrents()[0]);
        SmartDashboard.putBoolean("MotorIntakeAlgae Running", MotorIntakeAlgae.isRunning());
    }
}
