// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants;
import frc.robot.Constants.kDrivetrain.AlignPosition;
import frc.robot.Constants.kElevator.Level;
import frc.robot.Constants.kElevator.LevelType;
import frc.robot.AlignPositionSelector;
import frc.robot.LimelightHelpers;
import frc.robot.commands.Drivetrain.PoseAlignCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.drivetrain.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignAndScoreCoral extends SequentialCommandGroup {

  /** Creates a new AlignAndScoreCoral. */
  public AlignAndScoreCoral(Drivetrain drivetrain, Elevator elevator, EndEffector endEffector) {

    AlignPosition position = AlignPositionSelector.getSelectedAlignPosition();
    int targetTagID = DriverStation.getAlliance().get() == Alliance.Blue ? position.blueAprilTagID : position.redAprilTagID;
    PoseAlignCommand alignWithReef = new PoseAlignCommand(
      drivetrain,
      position.fieldPosition.plus(new Transform2d(
        new Translation2d(
          EndEffector.getCoralLevel() == Level.LEVEL4 ? 
            Constants.kDrivetrain.L4_X_DISTANCE_TO_REEF 
            : Constants.kDrivetrain.NON_L4_X_DISTANCE_TO_REEF, 
          position.yAlignPosition), 
        new Rotation2d())));
    ConditionalCommand moveToLevel = new ConditionalCommand(
      new MoveToLevel(endEffector, elevator, LevelType.CORAL, true), 
      new MoveToLevel(endEffector, elevator, LevelType.CORAL, false),
      () -> (Elevator.getCoralLevel().height - elevator.getPositions()[0] < 0));

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> drivetrain.setAligningWithReef(true)),
      AutoBuilder.pathfindToPoseFlipped(
        position.fieldPosition,
        Constants.kDrivetrain.PATH_CONSTRAINTS,
        0.5).until(
          () -> (LimelightHelpers.getFiducialID("limelight-left") == targetTagID 
            || LimelightHelpers.getFiducialID("limelight-right") == targetTagID)
              && alignWithReef.getDistanceFromTarget() <= 0.9),
      new InstantCommand(moveToLevel::schedule),
      alignWithReef);

  }
  
}
