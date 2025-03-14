// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Constants;
import frc.robot.Constants.kDrivetrain.CoralPosition;
import frc.robot.commands.Drivetrain.CoralPositionAlignCommand;
import frc.robot.commands.EndEffector.IndexSequence;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.drivetrain.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignAndGetCoralAutonomous extends SequentialCommandGroup {

  /** Creates a new AlignAndGetCoralAutonomous. */
  public AlignAndGetCoralAutonomous(Drivetrain drivetrain, Elevator elevator, EndEffector endEffector, CoralPosition position) {

    CoralPositionAlignCommand coralPositionAlign = new CoralPositionAlignCommand(
      drivetrain, 
      position, 
      Constants.kDrivetrain.X_DISTANCE_TO_CORAL_STATION);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      AutoBuilder.pathfindToPoseFlipped(
        position.fieldPosition,
        Constants.kDrivetrain.PATH_CONSTRAINTS,
        0.5).until(
          () -> drivetrain.getPose().getTranslation().getDistance(coralPositionAlign.getTargetPose().getTranslation()) <= 0.9),
      new InstantCommand(new IndexSequence(endEffector, elevator, null)::schedule),
      coralPositionAlign,
      new WaitCommand(1.5));

  }

}
