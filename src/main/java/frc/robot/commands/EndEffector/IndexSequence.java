// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.kElevator.Level;
import frc.robot.Constants.kElevator.LevelType;
import frc.robot.commands.Elevator.ManualElevator;
import frc.robot.commands.Elevator.MoveElevatorToLevel;
import frc.robot.commands.Scoring.SetLevel;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IndexSequence extends SequentialCommandGroup {
  /** Creates a new IndexSequence. */
  public IndexSequence(EndEffector endEffector, Elevator elevator, GenericHID controller) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    SequentialCommandGroup indexSequence = new SequentialCommandGroup(new IndexInCommand(endEffector, controller),
        new IndexAlignCommand(endEffector));
    ParallelDeadlineGroup indexAndManualElevator = new ParallelDeadlineGroup(indexSequence,
        new ManualElevator(elevator, controller));
    addCommands(new SetLevel(Level.SOURCE, LevelType.SOURCE),
        new ParallelCommandGroup(new MoveElevatorToLevel(elevator, LevelType.SOURCE),
            new PrepareEndEffector(endEffector, LevelType.SOURCE, false)),
        indexAndManualElevator);
  }
}
