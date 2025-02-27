// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.Elevator.ElevatorToAlgae;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.Constants.kElevator.Level;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickupAlgae extends ParallelCommandGroup {
  /** Creates a new PickupAlgae. */
  public PickupAlgae(Elevator elevator, Level level, EndEffector endEffector) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(new ElevatorToAlgae(elevator, level), new IntakeAlgae(endEffector));
  }
}
