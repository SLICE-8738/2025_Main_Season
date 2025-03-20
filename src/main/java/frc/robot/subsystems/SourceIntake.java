// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.slicelibs.PositionalSubsystem;
import frc.slicelibs.TalonFXPositionalSubsystem;

public class SourceIntake extends TalonFXPositionalSubsystem {

private final double DEFAULT_POSITION = 0;
private final DutyCycleEncoder m_absoluteEncoder;

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

    m_absoluteEncoder = new DutyCycleEncoder(Constants.kSourceIntake.ABSOLUTE_ENCODER_ID, Constants.kSourceIntake.ABSOLUTE_ENCODER_RANGE, 0);

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

  public void resetRelativeEncoder(){
    // if(m_absoluteEncoder.get() - (Constants.kSourceIntake.ABSOLUTE_ENCODER_OFFSET- 5) < 0){
    //   setEncoderPosition(m_absoluteEncoder.get() - Constants.kSourceIntake.ABSOLUTE_ENCODER_OFFSET + Constants.kSourceIntake.ABSOLUTE_ENCODER_RANGE);
    // } else{
    //   setEncoderPosition(m_absoluteEncoder.get() - Constants.kSourceIntake.ABSOLUTE_ENCODER_OFFSET);
    // }
    setEncoderPosition(DEFAULT_POSITION);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Source Intake Relative Angle", getPositions()[0]);
    SmartDashboard.putNumber("Source Absolute Angle", m_absoluteEncoder.get());

    Logger.recordOutput("Source Intake/Current Command", getCurrentCommand() == null ? "Nothing" : getCurrentCommand().getName());
  }
}
