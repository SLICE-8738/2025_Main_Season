// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kElevator.Level;
import frc.robot.commands.Elevator.MoveElevatorToLevel;
import frc.robot.commands.EndEffector.PrepareEndEffector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToStow extends SequentialCommandGroup {

  /** Creates a new ElevatorToSource. */
  public ToStow(EndEffector endEffector, Elevator elevator) {
    addCommands(new SetLevel(Level.STOW, endEffector), new ParallelCommandGroup(new PrepareEndEffector(endEffector), new MoveElevatorToLevel(elevator)).withTimeout(2.0));
  }
}
