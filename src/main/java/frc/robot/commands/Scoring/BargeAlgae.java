// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kElevator.Level;
import frc.robot.Constants.kElevator.LevelType;
import frc.robot.commands.EndEffector.ThrowAlgae;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BargeAlgae extends SequentialCommandGroup {
  /** Creates a new BargeAlgae. */
  public BargeAlgae(EndEffector endEffector, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetLevel(Level.BARGE, LevelType.SOURCE),
        new MoveToLevel(endEffector, elevator, LevelType.SOURCE, true),
        new SetLevel(Level.BARGE2, LevelType.SOURCE)
        );
  }
}
