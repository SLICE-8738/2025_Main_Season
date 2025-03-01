// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualElevator extends Command {
  private final Elevator m_elevator;
  private final GenericHID m_controller;

  /** Creates a new ManualElevator. */
  public ManualElevator(Elevator elevator, GenericHID controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);

    m_elevator = elevator;
    m_controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double axis = MathUtil.applyDeadband(-m_controller.getRawAxis(1), 0.1);

    if (axis == 0) {
      m_elevator.set(0.02);
    }
    else {
      m_elevator.set(axis);
    }
    // if (m_elevator.isAtBottom() && (axis > 0)) {
    //   m_elevator.set(0);
    // } else if (m_elevator.isAtTop() && (axis < 0)) {
    //   m_elevator.set(0);
    // }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.set(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
