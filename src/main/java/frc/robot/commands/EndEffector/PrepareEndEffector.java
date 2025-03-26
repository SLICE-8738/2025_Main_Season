// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.kElevator.Level;
import frc.robot.Constants.kElevator.LevelType;
import frc.robot.subsystems.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrepareEndEffector extends Command {
  /** Creates a new PrepareEndEffector. */
  EndEffector endEffector;
  double angle;
  private LevelType m_levelType;
  // true = up false = down
  private boolean movementDirection;
  private boolean m_retainAlgae;

  public PrepareEndEffector(EndEffector endEffector, LevelType levelType, boolean retainAlgae) {
    addRequirements(endEffector);
    this.endEffector = endEffector;
    m_levelType = levelType;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (m_levelType) {
      case SOURCE:
        angle = EndEffector.getSourceLevel().angle;
        break;
      case CORAL:
        angle = EndEffector.getCoralLevel().angle;
        break;
      case ALGAE:
        angle = EndEffector.getAlgaeLevel().angle;
        break;
    }

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
    if (angle == Level.SOURCE.angle) {
      endEffector.setPlacementMotor(-.1);
    }
    if (m_retainAlgae) {
      endEffector.setPlacementMotor(0.1);
    }
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
