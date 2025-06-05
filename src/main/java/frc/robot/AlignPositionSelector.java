package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import frc.robot.Constants.kField.AlignPosition;

public class AlignPositionSelector {

    private static final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");

    private static final ArrayList<GenericEntry> alignPositions = new ArrayList<GenericEntry>();
    private static GenericEntry selectedAlignPosition;

    public AlignPositionSelector() {

        /* Reef Positions */
        alignPositions.add(driverTab.add("", true).withPosition(5, 5).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        alignPositions.add(driverTab.add(" ", false).withPosition(4, 4).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        alignPositions.add(driverTab.add("  ", false).withPosition(3, 3).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        alignPositions.add(driverTab.add("   ", false).withPosition(3, 2).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        alignPositions.add(driverTab.add("    ", false).withPosition(4, 1).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        alignPositions.add(driverTab.add("     ", false).withPosition(5, 0).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        alignPositions.add(driverTab.add("      ", false).withPosition(6, 0).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        alignPositions.add(driverTab.add("       ", false).withPosition(7, 1).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        alignPositions.add(driverTab.add("        ", false).withPosition(8, 2).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        alignPositions.add(driverTab.add("         ", false).withPosition(8, 3).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        alignPositions.add(driverTab.add("          ", false).withPosition(7, 4).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        alignPositions.add(driverTab.add("           ", false).withPosition(6, 5).withWidget(BuiltInWidgets.kToggleButton).getEntry());

        selectedAlignPosition = alignPositions.get(0);

    }

    public void update() {

        for (int i = 0; i < alignPositions.size(); i++) {

            GenericEntry currentPosition = alignPositions.get(i);
            
            if (currentPosition.getBoolean(false) && currentPosition != selectedAlignPosition) {

                selectedAlignPosition.setBoolean(false);
                selectedAlignPosition = currentPosition;
                break;

            }
            else if (!currentPosition.getBoolean(false) && currentPosition == selectedAlignPosition) {

                selectedAlignPosition.setBoolean(true);
                break;

            }

        }

    }

    public static AlignPosition getSelectedAlignPosition() {

        return AlignPosition.values()[alignPositions.indexOf(selectedAlignPosition)];

    }

}
