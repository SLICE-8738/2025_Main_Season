// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SlowStow extends Command {
  /** Creates a new SlowStow. */
  private Elevator m_elevator;

  public SlowStow(Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.set(0.2); // Set elevator speed.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.set(0); // Reset the elevator speed to zero.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_elevator.getPositions()[0] <= Constants.kElevator.Level.STOW.height); // Ensures that it ends on the correct position.
  }
}
