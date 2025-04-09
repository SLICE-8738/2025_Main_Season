// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IndexInCommand extends Command {
  /** Creates a new EndEffectorCommand. */

  EndEffector m_endEffector;
  GenericHID m_controller;
  boolean frontSensor;
  boolean middleSensor;
  boolean topBackSensor;
  boolean bottomBackSensor;
  boolean maintaining;

  public IndexInCommand(EndEffector endEffector, GenericHID controller) {
    m_endEffector = endEffector;
    m_controller = controller;
    addRequirements(endEffector);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_endEffector.maintainPosition();
    maintaining = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean[] sensorGroup = EndEffector.checkSensorsIndexing();
    frontSensor = sensorGroup[0];
    middleSensor = sensorGroup[1];
    topBackSensor = sensorGroup[2];
    bottomBackSensor = sensorGroup[3];
    m_endEffector.setPlacementMotor(frontSensor ? -0.1 : -0.2); // Intake slower when the front sensor is activated

    // Manual control
    double axis = m_controller == null ? 0 : MathUtil.applyDeadband(m_controller.getRawAxis(0) * .5, .1);
    if ((axis < 0 && m_endEffector.getAngle().getDegrees() <= 0)
        || (axis > 0 && m_endEffector.getAngle().getDegrees() >= 84)) {
      m_endEffector.set(0);
      maintaining = false;
    } else if (axis == 0) {
      if (!maintaining) {
        m_endEffector.maintainPosition();
        maintaining = true;
      }
    } else {
      maintaining = false;
      m_endEffector.set(axis);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_endEffector.setPlacementMotor(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!topBackSensor && !bottomBackSensor && frontSensor && middleSensor);
  }
}
