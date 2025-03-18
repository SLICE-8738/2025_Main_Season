// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Button;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.slicelibs.PolarJoystickFilter;
import frc.slicelibs.util.JoystickFilterConfig;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralStationAlignCommand extends Command {

  private final Drivetrain m_drivetrain;

  private final PS4Controller m_driverController;

  private final PolarJoystickFilter translationFilter;

  private final PIDController rotationController;

  /** Creates a new CoralStationAlignCommand. */
  public CoralStationAlignCommand(Drivetrain drivetrain, PS4Controller driverController) {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;

    m_driverController = driverController;

    translationFilter = new PolarJoystickFilter(new JoystickFilterConfig(
        0.07,
        0.6,
        Constants.OperatorConstants.DRIVE_EXPONENT,
        Constants.OperatorConstants.DRIVE_EXPONENT_PERCENT));

    rotationController = new PIDController(4, 0, 0);
    rotationController.enableContinuousInput(0, 360);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    rotationController.setSetpoint(m_drivetrain.getClosestCoralStationRotation().getDegrees());

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double[] translation = translationFilter.filter(-m_driverController.getRawAxis(1), -m_driverController.getRawAxis(0));

    double multipler = Button.leftBumper1.getAsBoolean()? 0.33 : 1;

    double translationX = translation[0] * Constants.kDrivetrain.MAX_LINEAR_VELOCITY * multipler;
    double translationY = translation[1] * Constants.kDrivetrain.MAX_LINEAR_VELOCITY * multipler;
    double rotation = rotationController.calculate(m_drivetrain.getPose().getRotation().getDegrees());

    m_drivetrain.drive(
      new Transform2d(translationX, translationY, Rotation2d.fromDegrees(rotation)), 
      false, 
      true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_drivetrain.drive(
      new Transform2d(), 
      false,
      true);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
