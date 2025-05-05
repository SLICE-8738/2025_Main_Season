// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.drivetrain.Drivetrain;

/** Contains and runs all code needed to display all necessary information on Shuffleboard.*/
public class ShuffleboardData {

    private final ShuffleboardTab driverTab, debugTab, swerveTab, autoTab;

    public ShuffleboardData(Drivetrain drivetrain, EndEffector endEffector, AutoSelector autoSelector) {

        driverTab = Shuffleboard.getTab("Driver");
        debugTab = Shuffleboard.getTab("Debug");
        swerveTab = Shuffleboard.getTab("Swerve");
        autoTab = Shuffleboard.getTab("Autonomous");

        // ==========================
        // Drivetrain
        // ==========================

        //Displays the current velocity in meters per second of the left front swerve module on Shuffleboard
        swerveTab.addDouble("Front Left Velocity", () -> drivetrain.getModuleStates()[0].speedMetersPerSecond).
        withPosition(0, 0).
        withSize(2, 1);
        //Displays the current velocity in meters per second of the right front swerve module on Shuffleboard
        swerveTab.addDouble("Front Right Velocity", () -> drivetrain.getModuleStates()[1].speedMetersPerSecond).
        withPosition(7, 0).
        withSize(2, 1);
        //Displays the current velocity in meters per second of the right back swerve module on Shuffleboard
        swerveTab.addDouble("Back Right Velocity", () -> drivetrain.getModuleStates()[2].speedMetersPerSecond).
        withPosition(7, 3).
        withSize(2, 1);
        //Displays the current velocity in meters per second of the left back swerve module on Shuffleboard
        swerveTab.addDouble("Back Left Velocity", () -> drivetrain.getModuleStates()[3].speedMetersPerSecond).
        withPosition(0, 3).
        withSize(2, 1);
        
        //Displays the current absolute encoder angle in degrees with no offset of the left front swerve module on Shuffleboard
        swerveTab.addDouble("Front Left Absolute Angle", () -> drivetrain.getAbsoluteAngles()[0]).
        withWidget(BuiltInWidgets.kDial).
        withProperties(Map.of("Min", 0, "Max", 360)).
        withPosition(0, 1).
        withSize(2, 1);
        //Displays the current absolute encoder angle in degrees with no offset of the left back swerve module on Shuffleboard
        swerveTab.addDouble("Front Right Absolute Angle", () -> drivetrain.getAbsoluteAngles()[1]).
        withWidget(BuiltInWidgets.kDial).
        withProperties(Map.of("Min", 0,"Max", 360)).
        withPosition(7, 1).
        withSize(2, 1);
        //Displays the current absolute encoder angle in degrees with no offset of the right front swerve module on Shuffleboard
        swerveTab.addDouble("Back Right Absolute Angle", () -> drivetrain.getAbsoluteAngles()[2]).
        withWidget(BuiltInWidgets.kDial).
        withProperties(Map.of("Min", 0, "Max", 360)).
        withPosition(7, 2).
        withSize(2, 1);
        //Displays the current absolute encoder angle in degrees with no offset of the right back swerve module on Shuffleboard
        swerveTab.addDouble("Back Left Absolute Angle", () -> drivetrain.getAbsoluteAngles()[3]).
        withWidget(BuiltInWidgets.kDial).
        withProperties(Map.of("Min", 0, "Max", 360)).
        withPosition(0, 2).
        withSize(2, 1);
        
        //Displays the current integrated encoder angle in degrees of the left front swerve module on Shuffleboard
        swerveTab.addDouble("Front Left Integrated Angle", () -> drivetrain.getModuleStates()[0].angle.getDegrees()).
        withPosition(2, 0).
        withSize(2, 1);
        //Displays the current integrated encoder angle in degrees of the right front swerve module on Shuffleboard
        swerveTab.addDouble("Front Right Integrated Angle", () -> drivetrain.getModuleStates()[1].angle.getDegrees()).
        withPosition(5, 0).
        withSize(2, 1);
        //Displays the current integrated encoder angle in degrees of the right back swerve module on Shuffleboard
        swerveTab.addDouble("Back Right Integrated Angle", () -> drivetrain.getModuleStates()[2].angle.getDegrees()).
        withPosition(5, 3).
        withSize(2, 1);
        //Displays the current integrated encoder angle in degrees of the left back swerve module on Shuffleboard
        swerveTab.addDouble("Back Left Integrated Angle", () -> drivetrain.getModuleStates()[3].angle.getDegrees()).
        withPosition(2, 3).
        withSize(2, 1);
        
        //Displays the current heading of the robot in degrees on Shuffleboard
        debugTab.addDouble("Drivetrain Heading", () -> MathUtil.inputModulus(drivetrain.getPose().getRotation().getDegrees(), 0, 360)).
        withWidget(BuiltInWidgets.kDial).
        withProperties(Map.of("Min", 0, "Max", 360)).
        withPosition(0, 0).
        withSize(2, 1);
                
        //Displays the current position of the robot on the field on Shuffleboard
        debugTab.add(drivetrain.m_field2d).
        withPosition(3, 1).
        withSize(4, 3);

        driverTab.addCamera("Left Limelight", "limelight-left-1", "http://10.87.38.201").
        withPosition(0, 1).
        withSize(3, 3);
        driverTab.addCamera("Right Limelight", "limelight-right-1", "http://10.87.38.202").
        withPosition(9, 1).
        withSize(3, 3);

        debugTab.add("SysID Routine", drivetrain.sysIDChooser);

        // ==========================
        // Autonomous
        // ==========================

        //Adds the sendable chooser for the desired autonomous routine onto Shuffleboard
        autoTab.add("Auto Routine", autoSelector.routineChooser).
        withPosition(2, 0).
        withSize(2, 1);

        //Displays the autonomous routine selected on the sendable chooser on Shuffleboard
        autoTab.addString("Selected Auto Mode", autoSelector::getRoutine).
        withPosition(2, 1).
        withSize(2, 1);

        // ==========================
        // End Effector
        // ==========================

        ShuffleboardTuner.create(
            value -> endEffector.normalKG = value,
            "End Effector kG");
    }

}