// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IndexCommand extends Command {
  /** Creates a new EndEffectorCommand. */

  EndEffector endEffector;
  Boolean frontSensor;
  // Boolean middleSensor;
  Boolean backSensor;

  public IndexCommand(EndEffector endEffector) {
    this.endEffector = endEffector;
    addRequirements(endEffector);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Boolean[] sensorGroup = endEffector.checkSensorsIndexing();
    frontSensor = sensorGroup[0];
    // middleSensor = sensorGroup[1];
    backSensor = sensorGroup[2];
    endEffector.setPlacementMotor(-0.1);
    endEffector.antiGravity();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffector.setPlacementMotor(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (backSensor == false && frontSensor == true) {
      return true;
    }
    return false;
  }

  // && middleSensor == true
}
