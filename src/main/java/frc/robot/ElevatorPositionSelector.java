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
        private static final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
        private static final ArrayList<GenericEntry> positions = new ArrayList<GenericEntry>();
        private static GenericEntry selectedPosition;

        public ElevatorPositionSelector() {
                positions.add(
                                driverTab.add("Level 1", true).withPosition(13, 6)
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
        }

        public void setSelectedPosition(int level) {
                switch (level) {
                        case 1:
                                selectedPosition.setBoolean(false);
                                selectedPosition = positions.get(0);
                                selectedPosition.setBoolean(true);
                                break;
                        case 2:
                                selectedPosition.setBoolean(false);
                                selectedPosition = positions.get(1);
                                selectedPosition.setBoolean(true);
                                break;
                        case 3:
                                selectedPosition.setBoolean(false);
                                selectedPosition = positions.get(2);
                                selectedPosition.setBoolean(true);
                                break;
                        case 4:
                                selectedPosition.setBoolean(false);
                                selectedPosition = positions.get(3);
                                selectedPosition.setBoolean(true);
                                break;
                }
        }

        public int getSelectedPosition() {
                for (int i = 0; i < positions.size(); i++) {
                        if (positions.get(i).equals(selectedPosition)) {
                                int position = i;
                                return position;
                        }
                }
                return -1;
        }
}
