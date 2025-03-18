// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClampAlgae extends Command {
  EndEffector m_endEffector;
  double m_angle = 50;

  /** Creates a new ClampAlgae. */
  public ClampAlgae(EndEffector endEffector) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(endEffector);
    m_endEffector = endEffector;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_endEffector.setPlacementMotor(0.065);
    m_endEffector.setPosition(m_angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_endEffector.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_endEffector.getAngle().getDegrees() > m_angle) {
      return true;
    }
    return false;
  }
}
