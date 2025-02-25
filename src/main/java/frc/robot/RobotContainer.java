// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.Drivetrain.*;
import frc.robot.commands.Elevator.ElevatorToStow;
import frc.robot.commands.Elevator.ManualElevator;
import frc.robot.commands.Elevator.MoveToLevel;
import frc.robot.commands.Elevator.SetElevatorLevel;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.RealSwerveModuleIO;
import frc.robot.subsystems.drivetrain.SimSwerveModuleIO;
import frc.robot.subsystems.drivetrain.SwerveModuleIO;
import frc.robot.testing.routines.DrivetrainTest;
import frc.slicelibs.config.CTREConfigs;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final PS4Controller driverController = Button.controller1;
  private final GenericHID operatorController = Button.controller2;

  // ==========================
  // Subsystems
  // ==========================

  public final Drivetrain m_drivetrain;
  public final Elevator m_elevator;
  public final LEDs m_leds;

  public final AutoSelector m_autoSelector;
  public final CoralPositionSelector m_coralPositionSelector;
  public final ElevatorPositionSelector m_elevatorPositionSelector;
  public final ShuffleboardData m_shuffleboardData;

  // ==========================
  // Commands
  // ==========================

  /* Drivetrain */
  public final DriveCommand m_swerveDriveOpenLoop;
  public final DriveCommand m_swerveDriveClosedLoop;
  public final RunDutyCycleCommand m_setDrivePercentOutput;
  public final ResetFieldOrientedHeading m_resetFieldOrientedHeading;
  public final Command m_sysIDDriveRoutine;
  public final Command m_reefAlign;
  public final Command m_coralStationAlign;

  /* Elevator */
  public final ManualElevator m_manualElevator;
  public final MoveToLevel m_toLevel;
  public final SetElevatorLevel m_setLevelSource;
  public final SetElevatorLevel m_setLevelOne;
  public final SetElevatorLevel m_setLevelTwo;
  public final SetElevatorLevel m_setLevelThree;
  public final SetElevatorLevel m_setLevelFour;
  public final ElevatorToStow m_elevatorToStow;

  /* Tests */
  public final DrivetrainTest m_drivetrainTest;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // ==========================
    // Subsystems
    // ==========================

    switch (Constants.ADVANTAGE_KIT_MODE) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        m_drivetrain = new Drivetrain(
            new RealSwerveModuleIO(Constants.kDrivetrain.Mod0.CONSTANTS),
            new RealSwerveModuleIO(Constants.kDrivetrain.Mod1.CONSTANTS),
            new RealSwerveModuleIO(Constants.kDrivetrain.Mod2.CONSTANTS),
            new RealSwerveModuleIO(Constants.kDrivetrain.Mod3.CONSTANTS));
        break;
      case SIM:
        m_drivetrain = new Drivetrain(
            new SimSwerveModuleIO(),
            new SimSwerveModuleIO(),
            new SimSwerveModuleIO(),
            new SimSwerveModuleIO());
        break;
      default:
        m_drivetrain = new Drivetrain(
            new SwerveModuleIO() {
            },
            new SwerveModuleIO() {
            },
            new SwerveModuleIO() {
            },
            new SwerveModuleIO() {
            });
        break;
    }

    m_elevator = new Elevator(new int[] { Constants.kElevator.LEFT_MOTOR_ID, Constants.kElevator.RIGHT_MOTOR_ID },
        new boolean[] { true, false }, Constants.kElevator.KP, Constants.kElevator.KI, Constants.kElevator.KD,
        Constants.kElevator.POSITION_CONVERSION_FACTOR,
        Constants.kElevator.VELOCITY_CONVERSION_FACTOR);

    m_leds = new LEDs();

    m_autoSelector = new AutoSelector(m_drivetrain);
    m_reefPositionSelector = new ReefPositionSelector();
    m_elevatorPositionSelector = new ElevatorPositionSelector();
    m_coralPositionSelector = new CoralPositionSelector();
    m_elevatorPositionSelector = new ElevatorPositionSelector();
    m_shuffleboardData = new ShuffleboardData(m_drivetrain, m_autoSelector);

    // ==========================
    // Commands
    // ==========================

    /* Drivetrain */
    m_swerveDriveOpenLoop = new DriveCommand(m_drivetrain, driverController, true);
    m_swerveDriveClosedLoop = new DriveCommand(m_drivetrain, driverController, false);
    m_setDrivePercentOutput = new RunDutyCycleCommand(m_drivetrain, 0.10, 0);
    m_resetFieldOrientedHeading = new ResetFieldOrientedHeading(m_drivetrain);
    m_sysIDDriveRoutine = new DeferredCommand(m_drivetrain::getSysIDDriveRoutine, Set.of(m_drivetrain));
    m_reefAlign = new DeferredCommand(
        () -> AutoBuilder.pathfindToPoseFlipped(
            CoralPositionSelector.getSelectedReefFieldPosition(),
            Constants.kDrivetrain.PATH_CONSTRAINTS,
            0.5).andThen(new CoralPositionAlignCommand(m_drivetrain, driverController, true)),
        Set.of(m_drivetrain));
    m_coralStationAlign = new DeferredCommand(
        () -> AutoBuilder.pathfindToPoseFlipped(
            CoralPositionSelector.getSelectedCoralStationFieldPosition(),
            Constants.kDrivetrain.PATH_CONSTRAINTS,
            0.5).andThen(new CoralPositionAlignCommand(m_drivetrain, driverController, false)),
        Set.of(m_drivetrain));

    /* Elevator */

    m_manualElevator = new ManualElevator(m_elevator, operatorController);
    m_toLevel = new MoveToLevel(m_elevator, Constants.kElevator.THRESHOLD);
    m_setLevelSource = new SetElevatorLevel(0);
    m_setLevelOne = new SetElevatorLevel(1);
    m_setLevelTwo = new SetElevatorLevel(2);
    m_setLevelThree = new SetElevatorLevel(3);
    m_setLevelFour = new SetElevatorLevel(4);
    m_elevatorToStow = new ElevatorToStow(m_elevator, Constants.kElevator.THRESHOLD);

    /* Tests */
    m_drivetrainTest = new DrivetrainTest(m_drivetrain);

    // Configure the trigger bindings
    configureBindings();

    m_drivetrain.setDefaultCommand(m_swerveDriveClosedLoop);
    m_elevator.setDefaultCommand(m_manualElevator);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // ================
    // Driver Controls
    // ================

    /* Drivetrain */
    Button.triangle1.onTrue(m_resetFieldOrientedHeading);
    Button.controlPadLeft1.toggleOnTrue(m_sysIDDriveRoutine);
    Button.leftBumper1.whileTrue(m_reefAlign);
    Button.rightBumper1.whileTrue(m_coralStationAlign);
    /* Elevator */
    Button.rightTrigger1.onTrue(m_toLevel.withTimeout(2.0));
    Button.psButton1.onTrue(m_elevatorToStow);

    // ==================
    // Operator Controls
    // ==================

    /* Elevator */
    Button.controlPadDown2.onTrue(m_setLevelOne);
    Button.leftBumper2.onTrue(m_setLevelSource);
    Button.controlPadLeft2.onTrue(m_setLevelTwo);
    Button.controlPadRight2.onTrue(m_setLevelThree);
    Button.controlPadUp2.onTrue(m_setLevelFour);
    Button.start.onTrue(m_elevatorToStow);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return m_autoSelector.getAutoRoutine();

  }

}
