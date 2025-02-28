// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

/**
 * A command that runs the climber hooks until the climbing angle (defined in Constants) is reached.
 * This runs at full speed and thus should be run with caution.
 */
public class ClimbCommand extends Command {

  private final Climber climber;

  /** Creates a new Climb. */
  public ClimbCommand(Climber climber) {

    this.climber = climber;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.moveClimbMotor(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.moveClimbMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (climber.getPosition() - Constants.kClimber.CLIMB_POSITION) < 5;
  }
}
