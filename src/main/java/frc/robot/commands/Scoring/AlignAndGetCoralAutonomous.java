// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

//import java.util.List;
//import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
/*import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.util.FlippingUtil;*/

//import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
//import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Constants;
import frc.robot.Constants.kDrivetrain.AlignPosition;
import frc.robot.LimelightHelpers;
import frc.robot.commands.Drivetrain.PoseAlignCommand;
import frc.robot.commands.EndEffector.IndexSequence;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.drivetrain.Drivetrain;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignAndGetCoralAutonomous extends SequentialCommandGroup {

  /** Creates a new AlignAndGetCoralAutonomous. */
  public AlignAndGetCoralAutonomous(Drivetrain drivetrain, Elevator elevator, EndEffector endEffector, AlignPosition position) {

    int targetTagID = DriverStation.getAlliance().get() == Alliance.Blue ? position.blueAprilTagID : position.redAprilTagID;
    PoseAlignCommand alignWithCoralStation = new PoseAlignCommand(
      drivetrain,
      position.fieldPosition.plus(new Transform2d(
        new Translation2d(
          Constants.kDrivetrain.X_DISTANCE_TO_CORAL_STATION, 
          position.yAlignPosition), 
        new Rotation2d())));

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> drivetrain.setAligningWithReef(false)),
      AutoBuilder.pathfindToPoseFlipped(
        position.fieldPosition,
        Constants.kDrivetrain.PATH_CONSTRAINTS,
        0.5).until(() ->
          LimelightHelpers.getFiducialID("limelight-back") == targetTagID
            && alignWithCoralStation.getDistanceFromTarget() <= 0.9),
      /*new DeferredCommand(() -> {
        Pose2d midPoint = DriverStation.getAlliance().get() == Alliance.Blue ? 
          drivetrain.getLastCoralPosition().fieldPosition
          : FlippingUtil.flipFieldPose(drivetrain.getLastCoralPosition().fieldPosition);
        Pose2d endPoint = DriverStation.getAlliance().get() == Alliance.Blue ? 
          position.fieldPosition
          : FlippingUtil.flipFieldPose(position.fieldPosition);
  
        return AutoBuilder.followPath(PathPlannerPath.fromPathPoints(
          List.of(
            new PathPoint(drivetrain.getPose().getTranslation(), new RotationTarget(0, drivetrain.getPose().getRotation())),
            new PathPoint(midPoint.getTranslation(), new RotationTarget(0, midPoint.getRotation())),
            new PathPoint(endPoint.getTranslation())
          ),
          Constants.kDrivetrain.PATH_CONSTRAINTS,
          new GoalEndState(0.5, endPoint.getRotation())));},
        Set.of(drivetrain)).until(
          () -> drivetrain.getPose().getTranslation().getDistance(coralPositionAlign.getTargetPose().getTranslation()) <= 0.9)*/
      new InstantCommand(new IndexSequence(endEffector, elevator, null)::schedule),
      alignWithCoralStation,
      new WaitCommand(1.5));

  }

}
