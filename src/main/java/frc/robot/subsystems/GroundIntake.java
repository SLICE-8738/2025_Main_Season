// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.slicelibs.TalonFXPositionalSubsytem;

public class GroundIntake extends SubsystemBase {

  // TODO: FIND ACTUAL ID
  private TalonFX groundMotor;
  private double positionConversionFactor;
  private double velocityConversionFactor;
  private double positionTargetReference;
  private double velocityTargetReference;
  

  /** Creates a new GroundIntake. */
  public GroundIntake() {
    //TalonFX and TalonFX Configuration
    groundMotor = new TalonFX(0); // TODO - Change device ID.
    TalonFXConfiguration configuration = new TalonFXConfiguration();
    configuration.Slot0.kP = Constants.kGroundIntake.GROUND_INTAKE_KP;
    configuration.Slot0.kI = Constants.kGroundIntake.GROUND_INTAKE_KI;
    configuration.Slot0.kD = Constants.kGroundIntake.GROUND_INTAKE_KD;
    configuration.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    groundMotor.getConfigurator().apply(configuration);

    // Other needed in Constructor
    positionConversionFactor = Constants.kGroundIntake.POSITION_CONVERSION_FACTOR;
    velocityConversionFactor = Constants.kGroundIntake.VELOCITY_CONVERSION_FACTOR;
    
  }

  public void moveMotor(double speed){
    groundMotor.set(speed);
  }

  public double getVelocity(){
    double velocity = groundMotor.getVelocity().getValueAsDouble();
    return velocity;
  }

  public double getPosition(){
    double position = groundMotor.getPosition().getValueAsDouble();
    return position;
  }

  public void setVelocity(double set){
    VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

    groundMotor.setControl(velocityRequest.withVelocity(set * velocityConversionFactor));

    velocityTargetReference = set;

  }

  public void setPosition(double set){
    PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

    groundMotor.setControl(positionRequest.withPosition(set * positionConversionFactor);

    positionTargetReference = set;
  }

  public void setEncoderPosition(double set){
    groundMotor.getConfigurator().setPosition(set);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
