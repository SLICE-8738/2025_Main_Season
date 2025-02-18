// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundIntake extends SubsystemBase {

  private TalonFX groundMotor;
  private double positionConversionFactor;
  private double velocityConversionFactor;
  private double positionTargetReference;
  private double velocityTargetReference;

  /** Creates a new GroundIntake. */
  public GroundIntake(double kP, double kI, double kD, double PCF, double VCF) {
    //TalonFX and TalonFX Configuration
    groundMotor = new TalonFX(0); // TODO - Change device ID.
    TalonFXConfiguration configuration = new TalonFXConfiguration();
    configuration.Slot0.kP = kP;
    configuration.Slot0.kI = kI;
    configuration.Slot0.kD = kD;
    groundMotor.getConfigurator().apply(configuration);

    // Other needed in Constructor
    positionConversionFactor = PCF;
    velocityConversionFactor = VCF;
  }

  /**
   * Opens or closes the ground intake.
   */
  public void moveGroundIntake(double speed) {
    groundMotor.set(speed);
  }

  public void setVelocity(double velocity){
    VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    groundMotor.setControl(velocityRequest.withVelocity(velocity * velocityConversionFactor));
    velocityTargetReference = velocity;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
