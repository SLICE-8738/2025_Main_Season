// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** Add your docs here. */
public class ElevatorPositionSelector {
    private static final ShuffleboardTab driverTab = Shuffleboard.getTab("driver");
    private static final ArrayList<GenericEntry> positions = new ArrayList<GenericEntry>();
    private static GenericEntry selectedPosition;

    public ElevatorPositionSelector() {
        positions.add(
                driverTab.add("Level 1", true).withPosition(0, 3).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        positions.add(
                driverTab.add("Level 2", false).withPosition(0, 2).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        positions.add(
                driverTab.add("Level 3", false).withPosition(0, 1).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        positions.add(
                driverTab.add("Level 4", false).withPosition(0, 0).withWidget(BuiltInWidgets.kToggleButton).getEntry());

        selectedPosition = positions.get(0);
    }
}
