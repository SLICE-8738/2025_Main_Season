// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.slicelibs.PositionalSubsystem;
import frc.slicelibs.TalonFXPositionalSubsystem;



public class SourceIntake extends TalonFXPositionalSubsystem {

private final double DEFAULT_POSITION = 0;

  /** Creates a new SourceIntake. */
  public SourceIntake() {
    super(
      new int[] {Constants.kSourceIntake.MOTOR_PORT}, 
      new boolean[] {true}, 
      Constants.kSourceIntake.KP, 
      Constants.kSourceIntake.KI, 
      Constants.kSourceIntake.KD, 
      0.1,
      Constants.kSourceIntake.SENSOR_TO_MECHANISM_RATIO, 
      GravityTypeValue.Arm_Cosine,
      Constants.kSourceIntake.POSITION_CONVERSION_FACTOR, 
      Constants.kSourceIntake.VELOCITY_CONVERSION_FACTOR, 
      Constants.CTRE_CONFIGS.sourceIntakeFXConfig);

    setEncoderPosition(DEFAULT_POSITION);

  }

  public double getDefaultPosition(){
    return DEFAULT_POSITION;
  }

  /**
   * Sets the current position as a PID setpoint
   * and automatically applies anti-gravity feedforward
   */
  public void maintainPosition() {
    if (getPositionTargetReference() != getPositions()[0]) {
      setPosition(getPositions()[0]);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Source Intake Relative Angle", getPositions()[0]);
  }
}
