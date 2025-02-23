// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.hal.DIOJNI;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {
  private TalonFX rotationMotor;
  private TalonFX placementMotor;
  private AnalogEncoder encoder;
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
  private static DigitalInput middleSensor;

  // TODO fix static error

  /** Creates a new EndEffector. */
  public EndEffector() {
    rotationMotor = new TalonFX(Constants.kEndEffector.ROTATION_MOTOR_ID);
    placementMotor = new TalonFX(Constants.kEndEffector.PLACEMENT_MOTOR_ID);
    // TODO enter parameters
    frontSensor = new DigitalInput(1);
    backSensor = new DigitalInput(2);
    middleSensor = new DigitalInput(3);
  }
  

  public void setRotationMotorSpeed(double speed) {
    rotationMotor.set(speed);
  }

  public void setPlacementMotorSpeed(double speed) {
    placementMotor.set(speed);
  }



  

  public Boolean[] checkSensorsIndexing() {
    Boolean[] sensorStatuses = new Boolean[3];
    sensorStatuses[0] = frontSensor.get();
    sensorStatuses[1] = middleSensor.get();
    sensorStatuses[2] = backSensor.get();
    return sensorStatuses;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
