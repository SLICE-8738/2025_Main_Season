// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrepareEndEffector extends Command {
  /** Creates a new PrepareEndEffector. */
  EndEffector endEffector;
  double angle;
  public PrepareEndEffector(EndEffector endEffector, double angle) {
    this.endEffector = endEffector;
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endEffector.setPosition(angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffector.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (endEffector.atTarget(1) ) {
      return true;
    } else {
    return false;
    }
  }
}
