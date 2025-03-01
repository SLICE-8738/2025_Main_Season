// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.Constants.kDrivetrain.CoralPosition;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AutonomousCoralPositionAlignCommand extends Command {

  private final Drivetrain m_drivetrain;
  private final String side;

  private final PIDController xAlignController, zAlignController, rotationAlignController;

  /** Creates a new ReefAlignCommand. */
  public AutonomousCoralPositionAlignCommand(Drivetrain drivetrain, CoralPosition reefPosition) {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;
    side = reefPosition.side;

    xAlignController = new PIDController(2.5, 0, 0);
    zAlignController = new PIDController(1, 0, 0);
    rotationAlignController = new PIDController(2.5, 0, 0);

    xAlignController.setSetpoint(0);
    zAlignController.setSetpoint(Constants.kDrivetrain.ROBOT_FLUSH_SURFACE_Z_POSITION);
    rotationAlignController.setSetpoint(0);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double translationX = 0;
    double translationY = 0;
    double rotation = 0;

    if (LimelightHelpers.getTV("limelight-" + side)) {
      translationY = -xAlignController.calculate(LimelightHelpers.getTX("limelight-" + side));
      translationX = -zAlignController.calculate(LimelightHelpers.getBotPose3d_TargetSpace("limelight-" + side).getZ());
      rotation = -rotationAlignController.calculate(LimelightHelpers.getBotPose3d_TargetSpace("limelight-" + side).getRotation().getY());
    }

    m_drivetrain.drive(
      new Transform2d(translationX, translationY, new Rotation2d(rotation)), 
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
