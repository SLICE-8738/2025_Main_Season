// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EndEffectorCommand extends Command {
  /** Creates a new EndEffectorCommand. */
  EndEffector endEffector;
  Boolean frontMotor;
  Boolean middleMotor;
  Boolean backMotor;

  public EndEffectorCommand() {
    endEffector = new EndEffector();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Boolean[] sensorGroup = endEffector.checkSensorsIndexing();
    frontMotor = sensorGroup[0];
    middleMotor = sensorGroup[2];
    backMotor = sensorGroup[3];

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffector.setRotationMotorSpeed(0);
    endEffector.setPlacementMotorSpeed(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (backMotor == false) {
      return true;
    }
    return false;
  }
}
