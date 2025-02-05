// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SourceIntake;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SourceIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualRotateSourceIntake extends Command {
  private final SourceIntake m_sourceIntake;
  private final PS4Controller m_controller;
  private double axis;

  /** Creates a new ManualRotateSourceIntake. */
  public ManualRotateSourceIntake(SourceIntake intake, PS4Controller controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);

    m_sourceIntake = intake;
    m_controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    axis = m_controller.getRightX();
    m_sourceIntake.set(axis);
    if(m_sourceIntake.getPosition() <= m_sourceIntake.getDefaultPosition() && axis < 0){
      m_sourceIntake.set(0);
    }
    if(m_sourceIntake.getPosition() >= 180 && axis > 0){
      m_sourceIntake.set(0);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_sourceIntake.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
