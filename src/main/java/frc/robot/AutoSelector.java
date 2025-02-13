// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;

import frc.robot.Constants.kDrivetrain.CoralPosition;
import frc.robot.commands.Drivetrain.AutonomousCoralPositionAlignCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;

import java.util.Optional;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
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

        public final String name;

        StartingPosition(String name) {

            this.name = name;

        }

    }

    public enum Mode {

        TEST_PATH("Test Path", false),
        AUTO_BUILDER("Auto Builder", false);

        public final String name;
        public final boolean useStartingPosition;

        Mode(String name, boolean useStartingPosition) {

            this.name = name;
            this.useStartingPosition = useStartingPosition;

        }

    }

    private StartingPosition storedStartingPosition = StartingPosition.PLACEHOLDER;
    private Mode storedMode = Mode.TEST_PATH;

    public final SendableChooser<StartingPosition> startingPositionChooser;
    public final SendableChooser<Mode> modeChooser;

    private Optional<PathPlannerAuto> autoRoutine = Optional.empty();

    private Pose2d initialAutoPose = new Pose2d();

    private final Drivetrain m_drivetrain;

    public AutoSelector(Drivetrain drivetrain) {

        m_drivetrain = drivetrain;

        startingPositionChooser = new SendableChooser<StartingPosition>();

        startingPositionChooser.setDefaultOption("Placeholder", StartingPosition.PLACEHOLDER);

        startingPositionChooser.onChange((position) -> updateAutoRoutine(position, storedMode));

        modeChooser = new SendableChooser<Mode>();

        modeChooser.setDefaultOption(Mode.TEST_PATH.name, Mode.TEST_PATH);
        modeChooser.addOption(Mode.AUTO_BUILDER.name, Mode.AUTO_BUILDER);

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
                    DCMotor.getKrakenX60(1).withReduction(Constants.kDrivetrain.DRIVE_GEAR_RATIO),
                    Constants.kDrivetrain.DRIVE_STATOR_CURRENT_LIMIT,
                    1),
                Constants.kDrivetrain.kSwerveKinematics.getModules()
                ),
            () -> DriverStation.getAlliance().get() == Alliance.Red,
            m_drivetrain);

        for (CoralPosition position : CoralPosition.values()) {

            NamedCommands.registerCommand(
                "Go To" + position.name, 
                new DeferredCommand(() -> AutoBuilder.pathfindToPoseFlipped(
                    position.fieldPosition,
                    Constants.kDrivetrain.PATH_CONSTRAINTS,
                    0.5
                ).andThen(new AutonomousCoralPositionAlignCommand(drivetrain, position)),
                Set.of(drivetrain)));

        }

    }

    private void updateAutoRoutine(StartingPosition position, Mode mode) {

        storedStartingPosition = position;
        storedMode = mode;

        try {

            System.out.println("Auto selection changed, updating creator; Starting Position: " + position.name
                + ", Mode: " + mode.name);
            autoRoutine = Optional.of(new PathPlannerAuto(mode.useStartingPosition? position.name + " " + mode.name : mode.name));
            initialAutoPose = new PathPlannerAuto(mode.useStartingPosition? position.name + " " + mode.name : mode.name).getStartingPose();

        }
        catch (Exception e) {

            DriverStation.reportError(e.getMessage(), true);            
            autoRoutine = Optional.empty();

        }

    }

    public Transform2d getInitialAutoPoseOffset() {

        return initialAutoPose.minus(m_drivetrain.getPose());

    }

    public Command getAutoRoutine() {

        return autoRoutine.get();

    }

    public String getStartingPosition() {

        return startingPositionChooser.getSelected().name;

    }

    public String getMode() {

        return modeChooser.getSelected().name;

    }

}