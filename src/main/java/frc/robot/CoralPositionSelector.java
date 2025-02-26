package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import frc.robot.Constants.CoralPosition;

public class CoralPositionSelector {

    private static final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");

    private static final ArrayList<GenericEntry> reefPositions = new ArrayList<GenericEntry>();
    private static GenericEntry selectedReefPosition;
    private static final ArrayList<GenericEntry> coralStationPositions = new ArrayList<GenericEntry>();
    private static GenericEntry selectedCoralStationPosition;

    public CoralPositionSelector() {

        reefPositions.add(driverTab.add("", true).withPosition(5, 5).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        reefPositions.add(driverTab.add(" ", false).withPosition(4, 4).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        reefPositions.add(driverTab.add("  ", false).withPosition(3, 3).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        reefPositions.add(driverTab.add("   ", false).withPosition(3, 2).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        reefPositions.add(driverTab.add("    ", false).withPosition(4, 1).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        reefPositions.add(driverTab.add("     ", false).withPosition(5, 0).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        reefPositions.add(driverTab.add("      ", false).withPosition(6, 0).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        reefPositions.add(driverTab.add("       ", false).withPosition(7, 1).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        reefPositions.add(driverTab.add("        ", false).withPosition(8, 2).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        reefPositions.add(driverTab.add("         ", false).withPosition(8, 3).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        reefPositions.add(driverTab.add("          ", false).withPosition(7, 4).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        reefPositions.add(driverTab.add("           ", false).withPosition(6, 5).withWidget(BuiltInWidgets.kToggleButton).getEntry());

        coralStationPositions.add(driverTab.add("            ", true).withPosition(0, 5).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        coralStationPositions.add(driverTab.add("             ", false).withPosition(1, 5).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        coralStationPositions.add(driverTab.add("              ", false).withPosition(10, 5).withWidget(BuiltInWidgets.kToggleButton).getEntry());
        coralStationPositions.add(driverTab.add("               ", false).withPosition(11, 5).withWidget(BuiltInWidgets.kToggleButton).getEntry());

        selectedReefPosition = reefPositions.get(0);
        selectedCoralStationPosition = coralStationPositions.get(0);

    }

    public void update() {

        for (int i = 0; i < reefPositions.size(); i++) {

            GenericEntry currentPosition = reefPositions.get(i);
            
            if (currentPosition.getBoolean(false) && currentPosition != selectedReefPosition) {

                selectedReefPosition.setBoolean(false);
                selectedReefPosition = currentPosition;
                break;

            }

        }

        for (int i = 0; i < coralStationPositions.size(); i++) {

            GenericEntry currentPosition = coralStationPositions.get(i);

            if (currentPosition.getBoolean(false) && currentPosition != selectedCoralStationPosition) {

                selectedCoralStationPosition.setBoolean(false);
                selectedCoralStationPosition = currentPosition;
                break;

            }

        }

    }

    public static double getSelectedReefXAlignPosition() {

        return CoralPosition.values()[reefPositions.indexOf(selectedReefPosition)].xAlignPosition;

    }

    public static Pose2d getSelectedReefFieldPosition() {

        return CoralPosition.values()[reefPositions.indexOf(selectedReefPosition)].fieldPosition;

    }

    public static double getSelectedCoralStationXAlignPosition() {

        return CoralPosition.values()[coralStationPositions.indexOf(selectedCoralStationPosition) + 12].xAlignPosition;

    }

    public static Pose2d getSelectedCoralStationFieldPosition() {

        return CoralPosition.values()[coralStationPositions.indexOf(selectedCoralStationPosition) + 12].fieldPosition;

    }

}
