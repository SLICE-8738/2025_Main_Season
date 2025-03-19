// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.drivetrain.Drivetrain;

public class PoseAlignCommand extends Command {

  private final Drivetrain m_drivetrain;
  private final Pose2d m_targetPose;

  private final PIDController distanceController, rotationController;

  /**
   * Automatically aligns to the given pose using PID.
   * 
   * @param drivetrain The drivetrain subsystem instance passed in
   *                   from RobotContainer.
   * @param targetPose The pose to align to
   * @param automaticallyFlip Whether the given pose should automatically be
   *                          flipped to the red alliance side (given pose
   *                          must be for blue alliance)
   */
  public PoseAlignCommand(Drivetrain drivetrain, Pose2d targetPose) {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;
    m_targetPose = DriverStation.getAlliance().get() == Alliance.Blue ? targetPose : FlippingUtil.flipFieldPose(targetPose);

    distanceController = new PIDController(4, 0, 0);
    rotationController = new PIDController(3.5, 0, 0);

    distanceController.setSetpoint(0);
    distanceController.setTolerance(0.02);

    rotationController.setSetpoint(m_targetPose.getRotation().getDegrees());
    rotationController.enableContinuousInput(0, 360);
    rotationController.setTolerance(1);
        
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_drivetrain.addField2dPose(m_targetPose, "Auto Align Target Pose");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Translation2d difference = m_targetPose.minus(m_drivetrain.getPose()).getTranslation();
    double distanceFeedback = Math.abs(distanceController.calculate(Math.hypot(difference.getX(), difference.getY())));

    double translationX = difference.getAngle().getCos() * distanceFeedback;
    double translationY = difference.getAngle().getSin() * distanceFeedback;
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
    return distanceController.atSetpoint() && rotationController.atSetpoint();
  }

  public double getDistanceFromTarget() {
    return m_drivetrain.getPose().getTranslation().getDistance(m_targetPose.getTranslation());
  }

}
