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
  //TODO find actual IDs of Source Intake motors
  public SourceIntake() {
    super(new int[] {0, 1}, new boolean[] {true, false}, Constants.kSourceIntake.SOURCE_KP, Constants.kSourceIntake.SOURCE_KI, Constants.kSourceIntake.SOURCE_KD, GravityTypeValue.Arm_Cosine, Constants.kSourceIntake.SOURCE_INTAKE_POSITIONAL_CONVERSION_FACTOR, Constants.kSourceIntake.SOURCE_INTAKE_VELOCITY_CONVERSION_FACTOR, Constants.CTRE_CONFIGS.sourceIntakeFXConfig);
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
