// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/**
 * A command that moves the climber hook according to input axis 5 (up/down on right stick).
 * Intended to be used as a default command
 */
public class ManualClimberCommand extends Command {

  private final Climber climber;
  private final GenericHID controller;

  /** Creates a new ManualClimber. */
  public ManualClimberCommand(Climber climber, GenericHID controller) {

    this.climber = climber;
    this.controller = controller;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = controller.getRawAxis(5);
    climber.moveClimbMotor(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.moveClimbMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
