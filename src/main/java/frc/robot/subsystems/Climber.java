// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Climber extends SubsystemBase{

  private TalonFX climberMotor;
  private DutyCycleEncoder climberEncoder;

  /** Creates a new Climb. */
  public Climber() {
    climberMotor = new TalonFX(Constants.kClimber.MOTOR_ID); //TODO make sure to get actual device ID
    climberMotor.getConfigurator().apply(Constants.CTRE_CONFIGS.climberFXConfig);

    // The absolute encoder is used to get an accurate encoder reading no matter where the climber is on power-on
    climberEncoder = new DutyCycleEncoder(Constants.kClimber.ABSOLUTE_ENCODER_ID, 360, 0);
    climberMotor.getConfigurator().setPosition((climberEncoder.get()/360.0) - Constants.kClimber.ENCODER_OFFSET);
  }

  /**
   * Moves the climber hook. 
   * @param speed The speed to move the hook at, from -1 to 1. Positive is inwards (climbing), negative is outwards (unclimbing).
   */
  public void moveClimbMotor(double speed){
    climberMotor.set(speed);
  }

  /**
   * @return the climber's position in degrees. 0 is fully unclimbed, and positive is further inwards (more climbed).
   */
  public double getPosition() {
    return climberMotor.getPosition().getValueAsDouble() * 360.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
