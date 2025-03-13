// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.LEDs;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralLEDs extends Command {
  private final LEDs m_leds; // LEDs
  private final EndEffector m_endEffector; // End Effector for helping determine if coral is inside the robot

  private final Timer timer; // Timer for helping with blinkings
  
  // TODO Check HSV values
  private final Integer[] orange = {36, 245, 250}; // SLICE Orange (0xf9990a in RGB)
  private final Integer[] green = {120, 255, 255}; // Bright Green (0x00ff00 in RGB)

  /** Creates a new CoralLEDs. */
  public CoralLEDs(LEDs leds, EndEffector endEffector) {
    // Define LEDs and Timer
    m_leds = leds;
    timer = new Timer();
    m_endEffector = endEffector;

    addRequirements(leds);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_leds.setAllHSV(orange[0], orange[1], orange[2]); // Set the LEDs to orange initially.

    // Begin timer
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_endEffector.checkSensorsIndexing()[0] == true){ // We need the front sensor of the end effector to check the coral status.
      // Flash the lights green if the coral is within the robot
      if ((timer.get() % 1) > 0.5){
        m_leds.setAllHSV(green[0], green[1], green[2]); // Make it green
      }
      else {
        m_leds.setAll(Color.kBlack); // Make it black
      }
    }
    else {
      // If the coral is outside of the robot,
      m_leds.setAllHSV(orange[0], orange[1], orange[2]); // Keep the robot orange.
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_leds.setAllHSV(orange[0], orange[1], orange[2]); // Set it to orange when the command finishes.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
