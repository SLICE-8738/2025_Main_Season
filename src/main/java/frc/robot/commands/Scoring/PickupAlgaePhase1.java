// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.commands.EndEffector.MotorIntakeAlgae;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickupAlgaePhase1 extends SequentialCommandGroup {
  /** Creates a new PickupAlgaePhase1. */
  public PickupAlgaePhase1(Elevator elevator, EndEffector endEffector) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new MoveToLevelParallel(elevator, endEffector, Constants.kElevator.LevelType.ALGAE), new MotorIntakeAlgae(endEffector));
  }
}
