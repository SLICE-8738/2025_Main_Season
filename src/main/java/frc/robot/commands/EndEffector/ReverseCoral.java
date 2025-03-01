// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReverseCoral extends Command {
  private EndEffector m_endEffector;
  /** Creates a new ReverseCoral. */
  public ReverseCoral(EndEffector endEffector) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(endEffector);
    m_endEffector = endEffector;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_endEffector.setPlacementMotor(0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean finished = false;
    if(m_endEffector.checkSensorsIndexing()[2]){
      finished = true;
    }
    return finished;
  }
}
