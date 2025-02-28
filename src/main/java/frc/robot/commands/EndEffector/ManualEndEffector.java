// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualEndEffector extends Command {
  /** Creates a new ManualEndEffector. */
  EndEffector endEffector;
  GenericHID controller;

  public ManualEndEffector(EndEffector endEffector, GenericHID controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(endEffector);
    this.endEffector = endEffector;
    this.controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endEffector.resetRelativeEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double axis = MathUtil.applyDeadband(controller.getRawAxis(0) * .5, .1);
    if (axis < 0 && endEffector.getAngle().getDegrees() <= 0) {
      axis = 0;
    }
    if (axis > 0 && endEffector.getAngle().getDegrees() >= 88) {
      axis = 0;
    }
    if (axis == 0) {
      axis = .015 * endEffector.getAngle().times(2/3).getCos();
    }
    endEffector.set(axis);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
