// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
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
  private static CANrange middleSensor;
  private static CANrange topBackSensor;
  private static CANrange bottomBackSensor;

  private static boolean lastFront, lastMiddle, topLastBack, bottomLastBack;
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
    frontSensor = new CANrange(24);
    middleSensor = new CANrange(27); // change id
    topBackSensor = new CANrange(26);
    bottomBackSensor = new CANrange(25); // change id

    CANrangeConfiguration config = new CANrangeConfiguration();

    config.FovParams.FOVCenterX = 0;
    config.FovParams.FOVCenterY = 0;
    config.FovParams.FOVRangeX = 6.75;
    config.FovParams.FOVRangeY = 6.75;
    config.ProximityParams.ProximityThreshold = 0.13;
    config.ProximityParams.ProximityHysteresis = .001;
    config.ProximityParams.MinSignalStrengthForValidMeasurement = 15000;

    frontSensor.getConfigurator().apply(config);
    //config = config.withProximityParams((config.ProximityParams.withProximityThreshold(0.13)));
    // Middle and back sensor have offset centers to prevent them from triggering each other.
    middleSensor.getConfigurator().apply(config);
    topBackSensor.getConfigurator().apply(config);
    bottomBackSensor.getConfigurator().apply(config);



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
    boolean bottomBackSensor = sensors[3];
    boolean topBackSensor = sensors[2];
    boolean middleSensor = sensors[1];
    boolean frontSensor = sensors[0];

    if(!bottomBackSensor && !topBackSensor && !middleSensor && !frontSensor){
      setPlacementMotor(0);
    }
    else if(!topBackSensor && !bottomBackSensor && middleSensor && frontSensor){
      setPlacementMotor(0);
    } else if(!topBackSensor && !bottomBackSensor && !middleSensor && frontSensor){
      setPlacementMotor(0.05);
    }else{
      setPlacementMotor(-0.05);
    }
  }

  public static boolean[] checkSensorsIndexing() {
    boolean[] sensorStatuses = new boolean[4];
    sensorStatuses[0] = frontSensor.getIsDetected().getValue();
    sensorStatuses[1] = middleSensor.getIsDetected().getValue();
    sensorStatuses[2] = topBackSensor.getIsDetected().getValue();
    sensorStatuses[3] = bottomBackSensor.getIsDetected().getValue();

    double[] measurementTimes = new double[]{
      frontSensor.getMeasurementTime().getValueAsDouble(),
      middleSensor.getMeasurementTime().getValueAsDouble(), 
      topBackSensor.getMeasurementTime().getValueAsDouble(), 
      bottomBackSensor.getMeasurementTime().getValueAsDouble()};

    SmartDashboard.putNumber("Maxiumum Measurement Time", Math.max(Math.max(measurementTimes[0], measurementTimes[1]), Math.max(measurementTimes[2], measurementTimes[3])));

    lastFront = frontSensor.getIsDetected().getValue();
    lastMiddle = middleSensor.getIsDetected().getValue();
    topLastBack = topBackSensor.getIsDetected().getValue();
    bottomLastBack = bottomBackSensor.getIsDetected().getValue();

    Logger.recordOutput("End Effector/Front Sensor", sensorStatuses[0]);
    Logger.recordOutput("End Effector/Middle Sensor", sensorStatuses[1]);
    Logger.recordOutput("End Effector/Top Back Sensor", sensorStatuses[2]);
    Logger.recordOutput("End Effector/Bottom Back Sensor", sensorStatuses[3]);

    return sensorStatuses;
  }

  public void resetRelativeEncoder() {
    setEncoderPosition(360 + encoder.get() - Constants.kEndEffector.ENCODER_OFFSET);
    /*if (encoder.get() < 60) {
      setEncoderPosition(360 + encoder.get() - Constants.kEndEffector.ENCODER_OFFSET);
    } else {
      setEncoderPosition(encoder.get() - Constants.kEndEffector.ENCODER_OFFSET);
    }*/
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Absolute End Effector Angle", encoder.get());
    SmartDashboard.putNumber("Relative End Effector Angle", getPositions()[0]);
    SmartDashboard.putBoolean("SensorFront", checkSensorsIndexing()[0]);
    SmartDashboard.putBoolean("SensorBack", checkSensorsIndexing()[2]);
    SmartDashboard.putBoolean("SensorMiddle", checkSensorsIndexing()[1]);
    SmartDashboard.putNumber("SensorFront Distance", frontSensor.getDistance().getValueAsDouble());
    SmartDashboard.putNumber("Sensor Top Back Distance", topBackSensor.getDistance().getValueAsDouble());
    SmartDashboard.putNumber("Sensor Bottom Back Distance", bottomBackSensor.getDistance().getValueAsDouble());
    SmartDashboard.putNumber("SensorMiddle Distance", middleSensor.getDistance().getValueAsDouble());
    SmartDashboard.putString("Last Command", getCurrentCommand() == null ? "null" : getCurrentCommand().getName());

    SmartDashboard.putNumber("Subsystem Target Angle", getLevelType().equals(LevelType.CORAL) ? getCoralLevel().angle
        : getLevelType().equals(LevelType.ALGAE) ? getAlgaeLevel().angle : getSourceLevel().angle);
    SmartDashboard.putNumber("Motor Target Angle", getTargetPosition());

    Logger.recordOutput("End Effector/Current Command",
        getCurrentCommand() == null ? "Nothing" : getCurrentCommand().getName());
  }
}
