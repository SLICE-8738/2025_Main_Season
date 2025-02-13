// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.slicelibs.PositionalSubsystem;

public class Climb extends SubsystemBase{

  private TalonFX climberMotor;
  private Encoder climberEncoder;

  /** Creates a new Climb. */
  public Climb() {
    climberMotor = new TalonFX(0); //TODO make sure to get actual device ID
    climberEncoder = new Encoder(0, 0);
  }

  public void moveClimbMotor(double speed){
    climberMotor.set(speed);
  }

  public double returnVelocity(){
    return climberMotor.getVelocity();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
