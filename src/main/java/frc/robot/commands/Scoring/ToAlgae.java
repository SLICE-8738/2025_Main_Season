// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.EndEffector.IntakeAlgae;
import frc.robot.commands.EndEffector.MotorIntakeAlgae;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.Constants.kElevator.Level;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToAlgae extends SequentialCommandGroup {
  /** Creates a new PickupAlgae. */
  public ToAlgae(Elevator elevator, EndEffector endEffector, boolean endEffectorFirst) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    if (endEffector.getSelectedAngle() != Level.ALGAE1 && endEffector.getSelectedAngle() != Level.ALGAE2) {
      addCommands(new MoveToLevel(endEffector, elevator, endEffectorFirst));
    } else if (endEffectorFirst) {
      addCommands(new SetLevel(endEffector.getSelectedAngle(), endEffector),
          new MoveToLevel(endEffector, elevator, endEffectorFirst),
          new MotorIntakeAlgae(endEffector));
    } else {
      addCommands(new SetLevel(endEffector.getSelectedAngle(), endEffector),
          new MoveToLevel(endEffector, elevator, endEffectorFirst),
          new MotorIntakeAlgae(endEffector));
    }
  }
}
