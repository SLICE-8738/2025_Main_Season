// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrepareEndEffector extends Command {
  /** Creates a new PrepareEndEffector. */
  EndEffector endEffector;
  double angle;
  // true = up false = down
  private boolean movementDirection;

  public PrepareEndEffector(EndEffector endEffector) {
    addRequirements(endEffector);
    this.endEffector = endEffector;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.angle = endEffector.getSelectedAngle().angle;
    endEffector.setPosition(angle);
    if (endEffector.getAngle().getDegrees() < angle) { 
      movementDirection = true;
    }
    if (endEffector.getAngle().getDegrees() > angle) {
      movementDirection = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (endEffector.getAngle().getDegrees() >= 88 && movementDirection) {
      return true;
    } else if (endEffector.getAngle().getDegrees() <= 0 && !movementDirection) {
      return true;
    }

    if (endEffector.atTarget(1)) {
      return true;
    } else {
      return false;
    }
  }
}
