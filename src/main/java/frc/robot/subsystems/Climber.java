// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Climber extends SubsystemBase{

  private TalonFX climberMotor;
  private Encoder climberEncoder;

  /** Creates a new Climb. */
  public Climber() {
    climberMotor = new TalonFX(Constants.kClimber.MOTOR_ID); //TODO make sure to get actual device ID
    //climberEncoder = new Encoder(0, 0);
    climberEncoder = new Encoder(null, null);
  }

  public void moveClimbMotor(double speed){
    climberMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
