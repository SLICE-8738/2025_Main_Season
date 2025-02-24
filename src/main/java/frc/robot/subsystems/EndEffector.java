// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.hal.DIOJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.slicelibs.TalonFXPositionalSubsystem;

public class EndEffector extends TalonFXPositionalSubsystem {
  private TalonFX placementMotor;
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
  private static DigitalInput backSensor; // this one
  //private static DigitalInput middleSensor;

  // TODO fix static error

  /** Creates a new EndEffector. */
  public EndEffector() {
    super(new int[] {11}, new boolean[] {false}, 0.13, 0, 0, GravityTypeValue.Arm_Cosine, Constants.kEndEffector.POSITIONAL_CONVERSION_FACTOR, Constants.kEndEffector.VELOCITY_CONVERSTION_FACTOR);
    placementMotor = new TalonFX(Constants.kEndEffector.PLACEMENT_MOTOR_ID);
    // TODO enter parameters
    frontSensor = new DigitalInput(8);
    backSensor = new DigitalInput(9);
    //middleSensor = new DigitalInput(3);

    encoder = new DutyCycleEncoder(6);

    encoder.setInverted(true);
    placementMotor.getConfigurator().setPosition((encoder.get() * 360) - Constants.kEndEffector.ENCODER_OFFSET);

  }

  public void setPlacementMotorSpeed(double speed) {
    placementMotor.set(speed);
  }

  public Rotation2d getRelativeAngle() {
    return Rotation2d.fromDegrees(getPosition()[0]);
  }

  public Boolean[] checkSensorsIndexing() {
    Boolean[] sensorStatuses = new Boolean[3];
    sensorStatuses[0] = !frontSensor.get();
    //sensorStatuses[1] = middleSensor.get();
    sensorStatuses[2] = !backSensor.get();
    return sensorStatuses;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Absolute End Effector Angle", encoder.get());
    SmartDashboard.putNumber("Relative End Effector Angle", getPosition()[0]);
    SmartDashboard.putBoolean("SensorFront", frontSensor.get());
    SmartDashboard.putBoolean("SensorBack", backSensor.get());

    // This method will be called once per scheduler run
  }
}
