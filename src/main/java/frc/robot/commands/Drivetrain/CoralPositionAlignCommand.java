// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.CoralPositionSelector;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.slicelibs.PolarJoystickFilter;
import frc.slicelibs.config.JoystickFilterConfig;

public class CoralPositionAlignCommand extends Command {

  private final Drivetrain m_drivetrain;
  private final PS4Controller m_driverController;
  private final boolean m_alignWithReef;

  private final PolarJoystickFilter translationFilter, rotationFilter;

  private final PIDController xAlignController, rotationAlignController;

  private static final GenericEntry aligningWithAprilTag = 
    Shuffleboard.getTab("Driver").add("Aligning with April Tag", false)
    .withPosition(0,0).withSize(2, 2).getEntry();

  /** Creates a new ReefAlignCommand. */
  public CoralPositionAlignCommand(Drivetrain drivetrain, PS4Controller driverController, boolean alignWithReef) {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;
    m_driverController = driverController;
    m_alignWithReef = alignWithReef;

    translationFilter = new PolarJoystickFilter(new JoystickFilterConfig(
      0.07,
      0.6,
      Constants.OperatorConstants.DRIVE_EXPONENT,
      Constants.OperatorConstants.DRIVE_EXPONENT_PERCENT));
    rotationFilter = new PolarJoystickFilter(new JoystickFilterConfig(
      0.07,
      0.6,
      Constants.OperatorConstants.TURN_EXPONENT,
      Constants.OperatorConstants.TURN_EXPONENT_PERCENT));

    xAlignController = new PIDController(2.5, 0, 0);
    rotationAlignController = new PIDController(2.5, 0, 0);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    aligningWithAprilTag.setBoolean(true);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double translationX;
    double translationY;
    double rotation;

    if (LimelightHelpers.getTV("limelight-right")) {
      translationX = translationFilter.filter(-m_driverController.getRawAxis(1), 0)[0] * Constants.kDrivetrain.MAX_LINEAR_VELOCITY;
      translationY = -xAlignController.calculate(
        LimelightHelpers.getBotPose3d_TargetSpace("limelight-right").getX(), 
        m_alignWithReef ? CoralPositionSelector.getSelectedReefXAlignPosition() : CoralPositionSelector.getSelectedCoralStationXAlignPosition());
      rotation = -rotationAlignController.calculate(LimelightHelpers.getBotPose3d_TargetSpace("limelight-right").getRotation().getY(), 0);
    }
    else {
      double[] translation = translationFilter.filter(-m_driverController.getRawAxis(1), -m_driverController.getRawAxis(0));
      translationX = translation[0] * Constants.kDrivetrain.MAX_LINEAR_VELOCITY;
      translationY = translation[1] * Constants.kDrivetrain.MAX_LINEAR_VELOCITY;
      rotation = rotationFilter.filter(-m_driverController.getRawAxis(2), 0)[0] * Constants.kDrivetrain.MAX_ANGULAR_VELOCITY;
    }

    SmartDashboard.putNumber("Reef Align X Error", xAlignController.getError());
    SmartDashboard.putNumber("Reef Align Rotation Error", rotationAlignController.getError());

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
    
    aligningWithAprilTag.setBoolean(false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
