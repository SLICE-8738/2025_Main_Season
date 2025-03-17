// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kElevator.Level;
import frc.robot.Constants.kElevator.LevelType;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetLevel extends Command {
  private Level level;
  private LevelType levelType;

  /** Creates a new SetElevatorLevel. */
  public SetLevel(Level level, LevelType levelType) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.level = level;
    this.levelType = levelType;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (levelType) {
      case SOURCE:
        EndEffector.setSourceLevel(level);
        Elevator.setSourceLevel(level);
        break;
      case CORAL:
        EndEffector.setCoralLevel(level);
        Elevator.setCoralLevel(level);
        break;
      case ALGAE:
        EndEffector.setAlgaeLevel(level);
        Elevator.setAlgaeLevel(level);
        break;
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
    return true;
  }
}
