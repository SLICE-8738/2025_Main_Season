// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.kDrivetrain.CoralPosition;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class CoralPositionAlignCommand extends Command {

  private final Drivetrain m_drivetrain;
  private final Pose2d targetPose;

  private final PIDController distanceController, rotationController;

  private final Timer timer;

  public CoralPositionAlignCommand(Drivetrain drivetrain, CoralPosition position, double xDistance) {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;

    distanceController = new PIDController(3.5, 0, 0);
    rotationController = new PIDController(3.5, 0, 0);

    distanceController.setSetpoint(0);
    distanceController.setTolerance(0.02);

    rotationController.setSetpoint(position.fieldPosition.getRotation().getDegrees());
    rotationController.enableContinuousInput(0, 360);

    targetPose = DriverStation.getAlliance().get() == Alliance.Blue ? 
      position.fieldPosition.plus(new Transform2d(
        new Translation2d(xDistance, position.yAlignPosition), 
        new Rotation2d()))
      : FlippingUtil.flipFieldPose(position.fieldPosition).plus(new Transform2d(
        new Translation2d(xDistance, position.yAlignPosition), 
        new Rotation2d()));

    timer = new Timer();
        
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_drivetrain.addField2dPose(targetPose, "Auto Align Target Pose");
    timer.restart();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Transform2d difference = targetPose.minus(m_drivetrain.getPose());
    double distanceFeedback = Math.abs(distanceController.calculate(Math.hypot(difference.getX(), difference.getY())));

    double translationX = difference.getTranslation().getAngle().getCos() * distanceFeedback;
    double translationY = difference.getTranslation().getAngle().getSin() * distanceFeedback;
    double rotation = rotationController.calculate(m_drivetrain.getPose().getRotation().getDegrees());

    m_drivetrain.drive(
      new Transform2d(translationX, translationY, Rotation2d.fromDegrees(rotation)), 
      false, 
      false);

      SmartDashboard.putNumber("Auto Align Distance Error", distanceController.getError());
      SmartDashboard.putNumber("Auto Align Rotation Error", rotationController.getError());

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
    return distanceController.atSetpoint() && timer.hasElapsed(0.5);
  }

  public Pose2d getTargetPose() {
    return targetPose;
  }

}
