// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.kElevator.Level;

/** Add your docs here. */
public class ElevatorPositionSelector {
        private static final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
        private static final ArrayList<GenericEntry> positions = new ArrayList<GenericEntry>();
        private static GenericEntry selectedPosition;
        private static Level level;

        public ElevatorPositionSelector() {
                positions.add(
                                driverTab.add("Level 1", true).withPosition(13, 8)
                                                .withWidget(BuiltInWidgets.kToggleButton).getEntry());
                positions.add(
                                driverTab.add("Source", false).withPosition(13, 6)
                                                .withWidget(BuiltInWidgets.kToggleButton).getEntry());
                positions.add(
                                driverTab.add("Level 2", false).withPosition(13, 4)
                                                .withWidget(BuiltInWidgets.kToggleButton).getEntry());
                positions.add(
                                driverTab.add("Level 3", false).withPosition(13, 2)
                                                .withWidget(BuiltInWidgets.kToggleButton).getEntry());
                positions.add(
                                driverTab.add("Level 4", false).withPosition(13, 0)
                                                .withWidget(BuiltInWidgets.kToggleButton).getEntry());

                selectedPosition = positions.get(0);
                level = Level.STOW;
        }

        public static void setSelectedPosition(Level level) {
                switch (level) {
                        case LEVEL1:
                                selectedPosition.setBoolean(false);
                                selectedPosition = positions.get(0);
                                selectedPosition.setBoolean(true);
                                break;
                        case LEVEL2:
                                selectedPosition.setBoolean(false);
                                selectedPosition = positions.get(1);
                                selectedPosition.setBoolean(true);
                                break;
                        case LEVEL3:
                                selectedPosition.setBoolean(false);
                                selectedPosition = positions.get(2);
                                selectedPosition.setBoolean(true);
                                break;
                        case LEVEL4:
                                selectedPosition.setBoolean(false);
                                selectedPosition = positions.get(3);
                                selectedPosition.setBoolean(true);
                                break;
                }
                ElevatorPositionSelector.level = level;
                // SmartDashboard.putNumber("Elevator Level", level);
        }

        public static Level getSelectedPosition() {
                return ElevatorPositionSelector.level;
        }
}
