package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.ReefPosition;

public class ReefPositionSelector {

    private static final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    private static final ArrayList<GenericEntry> positions = new ArrayList<GenericEntry>();
    private static GenericEntry storedSelectedPosition;

    public ReefPositionSelector() {

        positions.add(driverTab.add("", true).withPosition(5, 5).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        positions.add(driverTab.add(" ", false).withPosition(4, 4).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        positions.add(driverTab.add("  ", false).withPosition(3, 3).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        positions.add(driverTab.add("   ", false).withPosition(3, 2).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        positions.add(driverTab.add("    ", false).withPosition(4, 1).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        positions.add(driverTab.add("     ", false).withPosition(5, 0).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        positions.add(driverTab.add("      ", false).withPosition(6, 0).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        positions.add(driverTab.add("       ", false).withPosition(7, 1).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        positions.add(driverTab.add("        ", false).withPosition(8, 2).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        positions.add(driverTab.add("         ", false).withPosition(8, 3).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        positions.add(driverTab.add("          ", false).withPosition(7, 4).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        positions.add(driverTab.add("           ", false).withPosition(6, 5).withWidget(BuiltInWidgets.kToggleButton).getEntry());

        storedSelectedPosition = positions.get(0);

    }

    public void update() {

        for (int i = 0; i < positions.size(); i++) {

            GenericEntry currentPosition = positions.get(i);
            
            if (currentPosition.getBoolean(false) && currentPosition != storedSelectedPosition) {

                storedSelectedPosition.setBoolean(false);
                storedSelectedPosition = currentPosition;
                break;

            }

        }

    }

    public static double getSelectedBranchXPosition() {

        return ReefPosition.values()[positions.indexOf(storedSelectedPosition)].branchXPosition;

    }

    public static Pose2d getSelectedFieldPosition() {

        return ReefPosition.values()[positions.indexOf(storedSelectedPosition)].fieldPosition;

    }

}
