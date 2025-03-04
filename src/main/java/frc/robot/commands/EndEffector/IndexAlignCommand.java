// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IndexAlignCommand extends Command {
  /** Creates a new EndEffectorCommand. */

  EndEffector endEffector;
  boolean frontSensor;
  boolean middleSensor;
  boolean backSensor;

  Timer timer;

  public IndexAlignCommand(EndEffector endEffector) {
    this.endEffector = endEffector;
    addRequirements(endEffector);

    timer = new Timer();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean[] sensorGroup = endEffector.checkSensorsIndexing();
    frontSensor = sensorGroup[0];
    middleSensor = sensorGroup[1];
    backSensor = sensorGroup[2];
    endEffector.alignCoral();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffector.setPlacementMotor(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Don't allow the command to end for the first 0.5 seconds
    if (timer.get() < 0.5) {
      return false;
    }
    if (backSensor == false && frontSensor == true && middleSensor == true) {
      return true;
    }
    return false;
  }

}
