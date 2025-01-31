// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SourceIntake extends SubsystemBase {

  private TalonFX climbPrepareMotor;

  /** Creates a new SourceIntake. */
  public SourceIntake() {
    climbPrepareMotor = new TalonFX(0); //TODO: Find Actual Device ID    
  }

  public void moveIntake(){
    climbPrepareMotor.set(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
