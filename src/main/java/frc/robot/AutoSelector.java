// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

/**
 * This class primarily manages the creation and updating of the autonomous mode
 * and starting position {@link SendableChooser sendable choosers} on Shuffleboard.
 */
public class AutoSelector {

    public enum StartingPosition {

        PLACEHOLDER("Placeholder");

        public final String value;

        StartingPosition(String value) {

            this.value = value;

        }

    }

    public enum Mode {

        TEST_PATH("Test Path", false);

        public final String value;
        public final boolean useStartingPosition;

        Mode(String value, boolean useStartingPosition) {

            this.value = value;
            this.useStartingPosition = useStartingPosition;

        }

    }

    private StartingPosition storedStartingPosition = StartingPosition.PLACEHOLDER;
    private Mode storedMode = Mode.TEST_PATH;

    public final SendableChooser<StartingPosition> startingPositionChooser;
    public final SendableChooser<Mode> modeChooser;

    private Optional<WrapperCommand> autoRoutine = Optional.empty();

    private Pose2d initialAutoPose;

    public double initialAutoPoseXOffset = 0;
    public double initialAutoPoseYOffset = 0;
    public double initialAutoPoseRotationOffset = 0;

    private final Drivetrain m_drivetrain;

    public AutoSelector(Drivetrain drivetrain) {

        m_drivetrain = drivetrain;

        startingPositionChooser = new SendableChooser<StartingPosition>();

        startingPositionChooser.setDefaultOption("Placeholder", StartingPosition.PLACEHOLDER);

        startingPositionChooser.onChange((position) -> updateAutoRoutine(position, storedMode));

        modeChooser = new SendableChooser<Mode>();

        modeChooser.setDefaultOption(Mode.TEST_PATH.value, Mode.TEST_PATH);

        modeChooser.onChange((mode) -> updateAutoRoutine(storedStartingPosition, mode));

        AutoBuilder.configure(
            m_drivetrain::getPose,
            m_drivetrain::resetOdometry,
            m_drivetrain::getChassisSpeeds,
            m_drivetrain::runChassisSpeeds,
            new PPHolonomicDriveController(
                new PIDConstants(Constants.kDrivetrain.TRANSLATION_KP),
                new PIDConstants(Constants.kDrivetrain.ROTATION_KP)),
            new RobotConfig(
                Constants.kDrivetrain.MASS,
                Constants.kDrivetrain.MOMENT_OF_INERTIA,
                new ModuleConfig(
                    Constants.kDrivetrain.WHEEL_DIAMETER / 2,
                    Constants.kDrivetrain.MAX_LINEAR_VELOCITY,
                    Constants.kDrivetrain.WHEEL_COEFFICIENT_OF_FRICTION,
                    DCMotor.getNEO(1).withReduction(Constants.kDrivetrain.DRIVE_GEAR_RATIO),
                    Constants.kDrivetrain.DRIVE_CURRENT_LIMIT,
                    1),
                Constants.kDrivetrain.TRACK_WIDTH
                ),
            () -> DriverStation.getAlliance().get() == Alliance.Red,
            m_drivetrain);

        // Register named commands here

        updateAutoRoutine(storedStartingPosition, storedMode);

    }

    private void updateAutoRoutine(StartingPosition position, Mode mode) {

        storedStartingPosition = position;
        storedMode = mode;

        try {

            System.out.println("Auto selection changed, updating creator; Starting Position: " + position.value
                + ", Mode: " + mode.value);
            autoRoutine = Optional.of(
                new PathPlannerAuto(mode.useStartingPosition? position.value + " " + mode.value : mode.value)
                .handleInterrupt(() -> m_drivetrain.runChassisSpeeds(new ChassisSpeeds())));
            initialAutoPose = new PathPlannerAuto(mode.useStartingPosition? position.value + " " + mode.value : mode.value).getStartingPose();

        }
        catch (Exception e) {

            DriverStation.reportError(e.getMessage(), true);            
            autoRoutine = Optional.empty();

        }

    }

    public void updateInitialAutoPoseOffset() {

        Pose2d currentPose = m_drivetrain.getPose();

        if (currentPose != null && initialAutoPose != null) {

            Transform2d offset = initialAutoPose.minus(currentPose);

            initialAutoPoseXOffset = offset.getX();
            initialAutoPoseYOffset = offset.getY();
            initialAutoPoseRotationOffset = offset.getRotation().getDegrees();

        }

    }

    public Command getAutoRoutine() {

        return autoRoutine.get();

    }

    public String getStartingPosition() {

        return startingPositionChooser.getSelected().value;

    }

    public String getMode() {

        return modeChooser.getSelected().value;

    }

}