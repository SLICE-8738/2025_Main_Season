// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundIntake extends SubsystemBase {
  private TalonFX groundMotor;
  private Encoder groundEncoder;

  /** Creates a new GroundIntake. */
  public GroundIntake() {
    groundMotor = new TalonFX(0); // TODO - Change device ID.
    groundEncoder = new Encoder(0, 0); // TODO - Change channels.
  }

  /**
   * Opens or closes the ground intake.
   */
  public void moveGroundIntake(double speed) {
    groundMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
