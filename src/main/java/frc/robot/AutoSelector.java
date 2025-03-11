// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
//import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.kDrivetrain.CoralPosition;
import frc.robot.Constants.kElevator.Level;
import frc.robot.Constants.kElevator.LevelType;
import frc.robot.commands.Drivetrain.CoralPositionAlignCommand;
import frc.robot.commands.Scoring.MoveToLevel;
import frc.robot.commands.Scoring.SetLevel;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.drivetrain.Drivetrain;

//import java.io.File;
//import java.io.FileReader;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

//import org.json.simple.JSONArray;
//import org.json.simple.JSONObject;
//import org.json.simple.parser.JSONParser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;

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

    private final Map<String, Pose2d> autoPoses = new HashMap<String, Pose2d>();

    private final Drivetrain m_drivetrain, m_simDrivetrain;

    public AutoSelector(Drivetrain drivetrain, Drivetrain simDrivetrain, Elevator elevator, EndEffector endEffector) {

        m_drivetrain = drivetrain;
        m_simDrivetrain = simDrivetrain;

        startingPositionChooser = new SendableChooser<StartingPosition>();

        startingPositionChooser.setDefaultOption("Placeholder", StartingPosition.PLACEHOLDER);

        startingPositionChooser.onChange((position) -> updateAutoRoutine(position, storedMode));

        modeChooser = new SendableChooser<Mode>();

        modeChooser.setDefaultOption(Mode.TEST_PATH.name, Mode.TEST_PATH);
        modeChooser.addOption(Mode.AUTO_BUILDER.name, Mode.AUTO_BUILDER);

        modeChooser.onChange((mode) -> updateAutoRoutine(storedStartingPosition, mode));

        AutoBuilder.configure(
            drivetrain::getPose,
            drivetrain::resetOdometry,
            drivetrain::getChassisSpeeds,
            drivetrain::runChassisSpeeds,
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
                Constants.kDrivetrain.kSwerveKinematics.getModules()),
            () -> DriverStation.getAlliance().get() == Alliance.Red,
            drivetrain);

        Pathfinding.ensureInitialized();

        /* Reef Positions */
        for (int i = 0; i < 12; i++) {

            CoralPosition position = CoralPosition.values()[i];
            int targetTagID = DriverStation.getAlliance().get() == Alliance.Blue ? position.blueAprilTagID : position.redAprilTagID;

            for (/*String*/ Level level : /*new String[] {"L4, Non-L4"}*/ Level.values()) {

                CoralPositionAlignCommand coralPositionAlign = new CoralPositionAlignCommand(
                    drivetrain, 
                    position, 
                    /*EndEffector.getCoralLevel() == Level.LEVEL4 ? Constants.kDrivetrain.L4_X_DISTANCE_TO_REEF : Constants.kDrivetrain.NON_L4_X_DISTANCE_TO_REEF*/Constants.kDrivetrain.X_DISTANCE_TO_REEF);

                NamedCommands.registerCommand(
                    "Score Coral " + position.name + " " + level.name,
                    new SequentialCommandGroup(
                        AutoBuilder.pathfindToPoseFlipped(
                            position.fieldPosition,
                            Constants.kDrivetrain.PATH_CONSTRAINTS,
                            0.5).until(
                                () -> LimelightHelpers.getFiducialID("limelight-left") == targetTagID 
                                  || LimelightHelpers.getFiducialID("limelight-right") == targetTagID),
                        new SetLevel(level, LevelType.CORAL),
                        new ParallelCommandGroup(
                            coralPositionAlign,
                            new WaitUntilCommand(() -> coralPositionAlign.getDistanceFromTarget() <= 0.9).andThen(new ConditionalCommand(
                                new MoveToLevel(endEffector, elevator, LevelType.CORAL, true), 
                                new MoveToLevel(endEffector, elevator, LevelType.CORAL, false),
                                () -> (Elevator.getCoralLevel().height - elevator.getPositions()[0] < 0))))));

                autoPoses.put("Score Coral " + position.name + " " + level.name, position.fieldPosition);

            }

        }

        /* Coral Station Positions */
        for (int i = 12; i < 16; i++) {

            CoralPosition position = CoralPosition.values()[i];
            
                NamedCommands.registerCommand(
                    "Get Coral " + position.name, 
                    AutoBuilder.pathfindToPoseFlipped(
                        position.fieldPosition,
                        Constants.kDrivetrain.PATH_CONSTRAINTS,
                        1).andThen(new CoralPositionAlignCommand(
                            drivetrain, 
                            position, 
                            Constants.kDrivetrain.X_DISTANCE_TO_CORAL_STATION)));

                autoPoses.put("Get Coral " + position.name, position.fieldPosition);

        }

    }

    private void updateAutoRoutine(StartingPosition position, Mode mode) {

        storedStartingPosition = position;
        storedMode = mode;

        try {

            System.out.println("Auto selection changed, updating creator; Starting Position: " + position.name
                + ", Mode: " + mode.name);
            autoRoutine = Optional.of(new PathPlannerAuto(mode.useStartingPosition? position.name + " " + mode.name : mode.name));
            initialAutoPose = (mode.useStartingPosition) ? 
                new PathPlannerAuto(mode.useStartingPosition? position.name + " " + mode.name : mode.name).getStartingPose()
                : null;

            /*if (m_simDrivetrain != null) {

                AutoBuilder.configure(
                    m_simDrivetrain::getPose,
                    m_simDrivetrain::resetOdometry,
                    m_simDrivetrain::getChassisSpeeds,
                    m_simDrivetrain::runChassisSpeeds,
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
                        Constants.kDrivetrain.kSwerveKinematics.getModules()),
                    () -> DriverStation.getAlliance().get() == Alliance.Red,
                    m_simDrivetrain);

            }

            JSONObject autoJSON = (JSONObject) new JSONParser().parse(new FileReader(new File(Filesystem.getDeployDirectory(), "pathplanner/autos/" + autoRoutine.get().getName() + ".auto")));
            JSONArray autoCommands = (JSONArray) ((JSONObject) ((JSONObject) (autoJSON.get("command"))).get("data")).get("commands");

            SequentialCommandGroup autoSequence = new SequentialCommandGroup() {
                @Override
                public boolean runsWhenDisabled() {
                    return true;
                }
            };
            
            for (int i = 0; i < autoCommands.size(); i++) {

                String pathName = (String) ((JSONObject) ((JSONObject) autoCommands.get(i)).get("data")).get("name");

                autoSequence.addCommands(AutoBuilder.pathfindToPoseFlipped(
                    autoPoses.get(pathName), 
                    Constants.kDrivetrain.PATH_CONSTRAINTS,
                    0.5));

            }

            //autoSequence.schedule();

            if (m_simDrivetrain != null) {
                
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
                        Constants.kDrivetrain.kSwerveKinematics.getModules()),
                    () -> DriverStation.getAlliance().get() == Alliance.Red,
                    m_drivetrain);

            }*/

        }
        catch (Exception e) {

            DriverStation.reportError(e.getMessage(), false);   
            e.printStackTrace();         
            autoRoutine = Optional.empty();

        }

    }

    public Transform2d getInitialAutoPoseOffset() {

        return initialAutoPose == null ? new Transform2d() : initialAutoPose.minus(m_drivetrain.getPose());

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