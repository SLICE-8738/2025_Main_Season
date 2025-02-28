// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kElevator.Level;
import frc.robot.commands.Scoring.MoveToLevel;
import frc.robot.commands.Scoring.SetLevel;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorToAlgae extends SequentialCommandGroup {
  /** Creates a new ElevatorToAlgae. */
  public ElevatorToAlgae(Elevator elevator, Level level, EndEffector endEffector) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetLevel(level, endEffector), new MoveToLevel(endEffector, elevator, false));
  }
}
