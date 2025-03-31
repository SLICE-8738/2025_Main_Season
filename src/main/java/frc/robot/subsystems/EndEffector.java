// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.kElevator.Level;
import frc.robot.Constants.kElevator.LevelType;
import frc.slicelibs.TalonFXPositionalSubsystem;

public class EndEffector extends TalonFXPositionalSubsystem {
  private DutyCycleEncoder encoder;
  // TODO rename maybe idk
  /*
   * HOW INDEXING CORAL WORKS
   * Coral begins indexing into the end effector and begins to trip the back
   * sensor
   * Coral continues being indexed and trips the middle sensor
   * Coral continues to index and trips the front sensor
   * Then untrips the back sensor
   * The untrips the middle sensor which signals that the coral has indexed too
   * far
   * The end effector spins opposite to bring coral back in and retrip middle
   * sensor
   * Coral successfully indexed
   */
  private static CANrange frontSensor;
  private static CANrange backSensor;
  private static CANrange middleSensor;
  private TalonFX placementMotor;
  private static Level m_coralLevel = Level.LEVEL1;
  private static Level m_algaeLevel = Level.ALGAE1;
  private static Level m_sourceLevel = Level.SOURCE;
  private static LevelType m_levelType = LevelType.SOURCE;

  // private static DigitalInput middleSensor;
  public double normalKG = 2;

  // TODO fix static error

  /** Creates a new EndEffector. */
  public EndEffector() {
    super(
        new int[] { Constants.kEndEffector.ROTATION_MOTOR_ID },
        new boolean[] { false },
        1.75, // 4.0,
        1.0,
        0.175,
        Constants.kEndEffector.KG,
        Constants.kEndEffector.SENSOR_TO_MECHANISM_RATIO,
        GravityTypeValue.Arm_Cosine,
        Constants.kEndEffector.POSITIONAL_CONVERSION_FACTOR,
        Constants.kEndEffector.VELOCITY_CONVERSTION_FACTOR,
        Constants.CTRE_CONFIGS.positionalFXConfig);

    // TODO enter parameters
    frontSensor = new CANrange(8); // change id
    backSensor = new CANrange(9); // change id
    middleSensor = new CANrange(5); // change id
    CANrangeConfiguration config = new CANrangeConfiguration();

    config.FovParams.FOVCenterX = 0;
    config.FovParams.FOVCenterY = 0;

    frontSensor.getConfigurator().apply(config);
    backSensor.getConfigurator().apply(config);
    middleSensor.getConfigurator().apply(config);

    placementMotor = new TalonFX(Constants.kEndEffector.PLACEMENT_MOTOR_ID);
    encoder = new DutyCycleEncoder(6, 360, 0);
    encoder.setInverted(true);

  }

  /**
   * Sets the current position as a PID setpoint
   * and automatically applies anti-gravity feedforward
   */
  public void maintainPosition() {
    if (getPositionTargetReference() != getAngle().getDegrees()) {
      setPosition(getAngle().getDegrees());
    }
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(getPositions()[0]);
  }

  public static Level getCoralLevel() {
    return m_coralLevel;
  }

  public static Level getAlgaeLevel() {
    return m_algaeLevel;
  }

  public static Level getSourceLevel() {
    return m_sourceLevel;
  }

  public static LevelType getLevelType() {
    return m_levelType;
  }

  public static void setCoralLevel(Level angle) {
    m_coralLevel = angle;
  }

  public static void setAlgaeLevel(Level angle) {
    m_algaeLevel = angle;
  }

  public static void setSourceLevel(Level angle) {
    m_sourceLevel = angle;
  }

  public static void setLevelType(LevelType levelType) {
    m_levelType = levelType;
  }

  public void setPlacementMotor(double speed) {
    placementMotor.set(speed);
  }

  public void alignCoral() {
    boolean[] sensors = checkSensorsIndexing();
    boolean backSensor = sensors[2];
    boolean middleSensor = sensors[1];
    boolean frontSensor = sensors[0];

    if (!frontSensor) {
      setPlacementMotor(0);
    } else if (backSensor) {
      setPlacementMotor(-0.05);
    } else if (!middleSensor) {
      setPlacementMotor(0.05);
    } else {
      setPlacementMotor(0);
    }
  }

  public static boolean[] checkSensorsIndexing() {
    boolean[] sensorStatuses = new boolean[3];
    sensorStatuses[0] = !frontSensor.getIsDetected().getValue();
    sensorStatuses[1] = !middleSensor.getIsDetected().getValue();
    sensorStatuses[2] = !backSensor.getIsDetected().getValue();
    return sensorStatuses;
  }

  public void resetRelativeEncoder() {
    if (encoder.get() < 60) {
      setEncoderPosition(360 + encoder.get() - Constants.kEndEffector.ENCODER_OFFSET);
    } else {
      setEncoderPosition(encoder.get() - Constants.kEndEffector.ENCODER_OFFSET);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Absolute End Effector Angle", encoder.get());
    SmartDashboard.putNumber("Relative End Effector Angle", getPositions()[0]);
    SmartDashboard.putBoolean("SensorFront", frontSensor.getIsDetected().getValue());
    SmartDashboard.putBoolean("SensorBack", backSensor.getIsDetected().getValue());
    SmartDashboard.putBoolean("SensorMiddle", middleSensor.getIsDetected().getValue());
    SmartDashboard.putString("Last Command", getCurrentCommand() == null ? "null" : getCurrentCommand().getName());

    SmartDashboard.putNumber("Subsystem Target Angle", getLevelType().equals(LevelType.CORAL) ? getCoralLevel().angle
        : getLevelType().equals(LevelType.ALGAE) ? getAlgaeLevel().angle : getSourceLevel().angle);
    SmartDashboard.putNumber("Motor Target Angle", getTargetPosition());

    Logger.recordOutput("End Effector/Current Command",
        getCurrentCommand() == null ? "Nothing" : getCurrentCommand().getName());
  }
}
