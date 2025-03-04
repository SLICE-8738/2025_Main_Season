// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.Constants;
import frc.robot.Constants.kDrivetrain.CoralPosition;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class CoralPositionAlignCommand extends Command {

  private final Drivetrain m_drivetrain;
  private final Pose2d targetPose;

  private final PIDController distanceController, rotationController;

  /** Creates a new CoralPositionAlignCommand. */
  public CoralPositionAlignCommand(Drivetrain drivetrain, CoralPosition position) {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;

    distanceController = new PIDController(3.5, 0, 0);
    rotationController = new PIDController(2.5, 0, 0);

    distanceController.setSetpoint(0);
    rotationController.setSetpoint(position.fieldPosition.getRotation().getDegrees());
    rotationController.enableContinuousInput(0, 360);

    targetPose = position.fieldPosition.plus(new Transform2d(
      new Translation2d(Constants.kDrivetrain.X_DISTANCE_TO_REEF, position.yAlignPosition), 
      new Rotation2d()));

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_drivetrain.addField2dPose(targetPose, "Auto Align Target Pose");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Transform2d difference = targetPose.minus(m_drivetrain.getPose());
    double distanceFeedback = Math.abs(distanceController.calculate(difference.getTranslation().getDistance(new Translation2d())));

    double translationX = difference.getTranslation().getAngle().getCos() * distanceFeedback;
    double translationY = difference.getTranslation().getAngle().getSin() * distanceFeedback;
    double rotation = rotationController.calculate(m_drivetrain.getPose().getRotation().getDegrees());

    m_drivetrain.drive(
      new Transform2d(translationX, translationY, Rotation2d.fromDegrees(rotation)), 
      false, 
      false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_drivetrain.drive(
      new Transform2d(), 
      false,
      false);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
