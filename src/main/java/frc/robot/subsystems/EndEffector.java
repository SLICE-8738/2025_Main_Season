// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {
  private TalonFX rotationMotor;
  private TalonFX placementMotor;

  /** Creates a new EndEffector. */
  public EndEffector() {
    rotationMotor = new TalonFX(Constants.kEndEffector.ROTATION_MOTOR_ID);
    placementMotor = new TalonFX(Constants.kEndEffector.PLACEMENT_MOTOR_ID);
  }

  public void setRotationMotorSpeed() {
    rotationMotor.set(.3);
  }

  public void setPlacementMotorSpeed() {
    placementMotor.set(.3);
  }

  public void stopRotationMotor() {
    rotationMotor.set(0);
  }

  public void stopPlacementMotor() {
    rotationMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
