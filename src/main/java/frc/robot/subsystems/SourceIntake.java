// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.signals.GravityTypeValue;

import frc.robot.Constants;
import frc.slicelibs.PositionalSubsystem;
import frc.slicelibs.TalonFXPositionalSubsystem;



public class SourceIntake extends TalonFXPositionalSubsystem {

private final double DEFAULT_POSITION = -35;

  /** Creates a new SourceIntake. */
  public SourceIntake(int[] ids, boolean[] inverted, double kP, double kI, double kD, double positionConversionFactor, double velocityConversionFactor) {
    super(ids, inverted, kP, kI, kD, GravityTypeValue.Arm_Cosine, positionConversionFactor, velocityConversionFactor, Constants.CTRE_CONFIGS.sourceIntakeFXConfig);
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
