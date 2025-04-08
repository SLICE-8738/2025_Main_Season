// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants;
import frc.robot.Constants.kDrivetrain.AlignPosition;
import frc.robot.commands.Drivetrain.PoseAlignCommand;
import frc.robot.commands.Drivetrain.SetAligningWithReefCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.drivetrain.Drivetrain;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignAndGetCoral extends SequentialCommandGroup {

  /** Creates a new AlignAndGetCoral. */
  public AlignAndGetCoral(Drivetrain drivetrain, Elevator elevator, EndEffector endEffector, GenericHID controller) {

    AlignPosition position = drivetrain.getClosestCoralStationPosition();
    PoseAlignCommand alignWithCoralStation = new PoseAlignCommand(
      drivetrain,
      position.fieldPosition.plus(new Transform2d(
        new Translation2d(
          Constants.kDrivetrain.X_DISTANCE_TO_CORAL_STATION, 
          position.yAlignDistance.getAsDouble()), 
        new Rotation2d())));

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetAligningWithReefCommand(drivetrain, false),
      AutoBuilder.pathfindToPoseFlipped(
        position.fieldPosition,
        Constants.kDrivetrain.PATH_CONSTRAINTS,
        /*0.5*/ 0.3).until(() -> alignWithCoralStation.getDistanceFromTarget() <= 1.1),
      alignWithCoralStation);

  }

}
