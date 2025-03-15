// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.SourceIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ResetRelativeEncoders extends Command {
  /** Creates a new ResetRelativeEncoders. */
  private EndEffector m_endEffector;
  private SourceIntake m_sourceIntake;

  public ResetRelativeEncoders(EndEffector endEffector, SourceIntake sourceIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_endEffector = endEffector;
    m_sourceIntake = sourceIntake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_endEffector.resetRelativeEncoder();
    m_sourceIntake.resetRelativeEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
