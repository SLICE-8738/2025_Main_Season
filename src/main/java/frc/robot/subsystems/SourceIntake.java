// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.slicelibs.PositionalSubsystem;



public class SourceIntake extends PositionalSubsystem {

private final double DEFAULT_POSITION = -35;

  /** Creates a new SourceIntake. */
  public SourceIntake(int[] ids, boolean[] inverted, double kP, double kI, double kD, double positionConversionFactor, double velocityConversionFactor) {
    super(ids, inverted, kP, kI, kD, positionConversionFactor, velocityConversionFactor);
    setEncoderPosition(DEFAULT_POSITION);

  }

  public double getDefaultPosition(){
    return DEFAULT_POSITION;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
