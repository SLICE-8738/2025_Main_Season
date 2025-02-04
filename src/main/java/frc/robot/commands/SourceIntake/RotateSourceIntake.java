// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SourceIntake;

import java.nio.channels.Pipe.SourceChannel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SourceIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateSourceIntake extends Command {
  /** Creates a new RotateSourceIntake. */
  private final SourceIntake m_sourceIntake;
  private final double m_threshold;
  private final double m_degrees;

  public RotateSourceIntake(SourceIntake sourceIntake, double threshold, double degrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sourceIntake);

    m_sourceIntake = sourceIntake;
    m_threshold = threshold;
    m_degrees = degrees;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_sourceIntake.moveIntake(m_degrees);
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
    return m_sourceIntake.atTarget(m_threshold);
  }
}
