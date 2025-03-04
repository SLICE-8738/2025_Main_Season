// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.kElevator.Level;
import frc.robot.Constants;
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
  private static DigitalInput frontSensor;
  private static DigitalInput backSensor; 
  private static DigitalInput middleSensor;
  private TalonFX placementMotor;
  private Level m_angle = Level.STOW;
  // private static DigitalInput middleSensor;
  public double normalKG = 2;

  // TODO fix static error

  /** Creates a new EndEffector. */
  public EndEffector() {
    super(
        new int[] { Constants.kEndEffector.ROTATION_MOTOR_ID },
        new boolean[] { false },
        1.75,//4.0,
        1.0,
        0.175,
        Constants.kEndEffector.KG,
        Constants.kEndEffector.SENSOR_TO_MECHANISM_RATIO,
        GravityTypeValue.Arm_Cosine,
        Constants.kEndEffector.POSITIONAL_CONVERSION_FACTOR,
        Constants.kEndEffector.VELOCITY_CONVERSTION_FACTOR,
        Constants.CTRE_CONFIGS.positionalFXConfig);

    // TODO enter parameters
    frontSensor = new DigitalInput(8);
    backSensor = new DigitalInput(9);
    middleSensor = new DigitalInput(5);
    placementMotor = new TalonFX(Constants.kEndEffector.PLACEMENT_MOTOR_ID);
    // middleSensor = new DigitalInput(3);

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
    return Rotation2d.fromDegrees(getPosition()[0]);
  }

  public Level getSelectedAngle() {
    return m_angle;
  }

  public void setSelectedLevel(Level angle) {
    m_angle = angle;
  }

  public void setPlacementMotor(double speed) {
    placementMotor.set(speed);
  }

  public Boolean[] checkSensorsIndexing() {
    Boolean[] sensorStatuses = new Boolean[3];
    sensorStatuses[0] = !frontSensor.get();
    sensorStatuses[1] = !middleSensor.get();
    sensorStatuses[2] = !backSensor.get();
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
    SmartDashboard.putNumber("Absolute End Effector Angle", encoder.get());
    SmartDashboard.putNumber("Relative End Effector Angle", getPosition()[0]);
    SmartDashboard.putBoolean("SensorFront", frontSensor.get());
    SmartDashboard.putBoolean("SensorBack", backSensor.get());
    SmartDashboard.putBoolean("SensorMiddle", middleSensor.get());
    SmartDashboard.putNumber("End Effector Target", getSelectedAngle().angle);

    // This method will be called once per scheduler run
  }
}
