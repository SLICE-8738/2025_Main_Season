// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Climber.ClimbCommand;
import frc.robot.commands.Climber.ManualClimberCommand;
import frc.robot.Constants.kElevator.Level;
import frc.robot.commands.Drivetrain.*;
import frc.robot.commands.Elevator.ManualElevator;
import frc.robot.commands.EndEffector.BumpAlgae;
import frc.robot.commands.EndEffector.IndexCommand;
import frc.robot.commands.EndEffector.IntakeAlgae;
import frc.robot.commands.EndEffector.ManualEndEffector;
import frc.robot.commands.EndEffector.OutakeAlgae;
import frc.robot.commands.EndEffector.PrepareEndEffector;
import frc.robot.commands.EndEffector.ReverseCoral;
import frc.robot.commands.EndEffector.ScoreCoral;
import frc.robot.commands.Scoring.ToStow;
import frc.robot.commands.SourceIntake.ManualRotateSourceIntake;
import frc.robot.commands.SourceIntake.RotateSourceIntake;
import frc.robot.commands.Scoring.MoveToLevel;
import frc.robot.commands.Scoring.PickupAlgae;
import frc.robot.commands.Scoring.ScoreAlgae;
import frc.robot.commands.Scoring.SetLevel;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.RealSwerveModuleIO;
import frc.robot.subsystems.drivetrain.SimSwerveModuleIO;
import frc.robot.subsystems.drivetrain.SwerveModuleIO;
import frc.robot.testing.routines.DrivetrainTest;

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

  // public final Drivetrain m_drivetrain;
  // public final LEDs m_leds;

  // public final AutoSelector m_autoSelector;
  // public final ShuffleboardData m_shuffleboardData;
  public final Drivetrain m_drivetrain;
  public final Climber m_climber;
  public final Elevator m_elevator;
  public final EndEffector m_endEffector;
  public final SourceIntake m_sourceIntake;
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

  /* Scoring */
  public final SetLevel m_setLevelSource;
  public final SetLevel m_setLevelOne;
  public final SetLevel m_setLevelTwo;
  public final SetLevel m_setLevelThree;
  public final SetLevel m_setLevelFour;
  public final MoveToLevel m_moveUpToLevel;
  public final MoveToLevel m_moveDownToLevel;
  public final PickupAlgae m_pickupAlgaeUp1;
  public final PickupAlgae m_pickupAlgaeDown1;
  public final PickupAlgae m_pickupAlgaeUp2;
  public final PickupAlgae m_pickupAlgaeDown2;
  public final ScoreAlgae m_scoreAlgae;
  public final ToStow m_elevatorToStow;

  /* Climber */
  public final ManualClimberCommand m_manualClimb;
  public final SequentialCommandGroup m_climb;
  /* End Effector */
  public final IndexCommand m_indexCoral;
  public final BumpAlgae m_bumpAlgae;
  public final ScoreCoral m_scoreCoral;
  public final ManualEndEffector m_manualEndEffector;
  public final PrepareEndEffector m_testEndEffector;

  public final IntakeAlgae m_IntakeAlgae;
  public final OutakeAlgae m_OutakeAlgae;

  /* Source Intake */
  public final ManualRotateSourceIntake m_manualSourceIntake;
  public final RotateSourceIntake m_goToSourceIntakeAngle1;
  public final RotateSourceIntake m_goToSourceIntakeAngle2;

  /* Tests */
  public final DrivetrainTest m_drivetrainTest;
  public final InstantCommand m_antiGravityTest;

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
        m_autoSelector = new AutoSelector(
            m_drivetrain,
            /*new Drivetrain(
                new SimSwerveModuleIO(),
                new SimSwerveModuleIO(),
                new SimSwerveModuleIO(),
                new SimSwerveModuleIO())*/
                null);
        break;
      case SIM:
        m_drivetrain = new Drivetrain(
            new SimSwerveModuleIO(),
            new SimSwerveModuleIO(),
            new SimSwerveModuleIO(),
            new SimSwerveModuleIO());
        m_autoSelector = new AutoSelector(m_drivetrain, null);
        break;
      default:
        m_drivetrain = new Drivetrain(
            new SwerveModuleIO() {},
            new SwerveModuleIO() {},
            new SwerveModuleIO() {},
            new SwerveModuleIO() {});
        m_autoSelector = new AutoSelector(m_drivetrain, null);
        break;
    }

    m_climber = new Climber();
    m_elevator = new Elevator();
    m_endEffector = new EndEffector();
    m_sourceIntake = new SourceIntake();
    

    m_leds = new LEDs();

    m_coralPositionSelector = new CoralPositionSelector();
    m_elevatorPositionSelector = new ElevatorPositionSelector();
    m_shuffleboardData = new ShuffleboardData(m_drivetrain, m_endEffector, m_autoSelector);

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
          CoralPositionSelector.getSelectedReefPosition().fieldPosition,
          Constants.kDrivetrain.PATH_CONSTRAINTS,
          0.5).andThen(new CoralPositionAlignCommand(m_drivetrain, CoralPositionSelector.getSelectedReefPosition())),
      Set.of(m_drivetrain));
    m_coralStationAlign = new DeferredCommand(
      () -> AutoBuilder.pathfindToPoseFlipped(
          CoralPositionSelector.getSelectedCoralStationPosition().fieldPosition,
          Constants.kDrivetrain.PATH_CONSTRAINTS,
          0.5).andThen(new CoralPositionAlignCommand(m_drivetrain, CoralPositionSelector.getSelectedCoralStationPosition())),
      Set.of(m_drivetrain));

    /* End Effector */
    m_indexCoral = new IndexCommand(m_endEffector);
    m_bumpAlgae = new BumpAlgae(m_endEffector);
    m_scoreCoral = new ScoreCoral(m_endEffector);
    m_manualEndEffector = new ManualEndEffector(m_endEffector, operatorController);
    m_testEndEffector = new PrepareEndEffector(m_endEffector);

    m_IntakeAlgae = new IntakeAlgae(m_endEffector);
    m_OutakeAlgae = new OutakeAlgae(m_endEffector);

    /* Elevator */
    m_manualElevator = new ManualElevator(m_elevator, operatorController);

    /* Source Intake */
    m_manualSourceIntake = new ManualRotateSourceIntake(m_sourceIntake, operatorController);
    m_goToSourceIntakeAngle1 = new RotateSourceIntake(m_sourceIntake, 2, Constants.kSourceIntake.INTAKE_ANGLE);
    m_goToSourceIntakeAngle2 = new RotateSourceIntake(m_sourceIntake, 2, Constants.kSourceIntake.CLIMB_ANGLE);


    /* Scoring */
    m_moveUpToLevel = new MoveToLevel(m_endEffector, m_elevator, false);
    m_moveDownToLevel = new MoveToLevel(m_endEffector, m_elevator, true);
    m_setLevelSource = new SetLevel(Level.SOURCE, m_endEffector);
    m_setLevelOne = new SetLevel(Level.LEVEL1, m_endEffector);
    m_setLevelTwo = new SetLevel(Level.LEVEL2, m_endEffector);
    m_setLevelThree = new SetLevel(Level.LEVEL3, m_endEffector);
    m_setLevelFour = new SetLevel(Level.LEVEL4, m_endEffector);
    m_elevatorToStow = new ToStow(m_endEffector, m_elevator);
    m_pickupAlgaeUp1 = new PickupAlgae(m_elevator, Level.ALGAE1, m_endEffector, false);
    m_pickupAlgaeDown1 = new PickupAlgae(m_elevator, Level.ALGAE1, m_endEffector, true);
    m_pickupAlgaeUp2 = new PickupAlgae(m_elevator, Level.ALGAE2, m_endEffector, false);
    m_pickupAlgaeDown2 = new PickupAlgae(m_elevator, Level.ALGAE2, m_endEffector, true);
    m_scoreAlgae = new ScoreAlgae(m_elevator, m_endEffector);

    /* Climber */
    m_manualClimb = new ManualClimberCommand(m_climber, Button.controller2);
    // The climb command is created with a WaitCommand before it so that the climber
    // won't immediately activate when the button is pressed.
    // This prevents accidental damage to the climber by running it when there isn't
    // a cage.
    m_climb = new SequentialCommandGroup(new WaitCommand(0.5), new ClimbCommand(m_climber));

    /* Tests */
    m_drivetrainTest = new DrivetrainTest(m_drivetrain);
    m_antiGravityTest = new InstantCommand(() -> m_endEffector.setVoltage(-m_endEffector.normalKG), m_endEffector);

    // Configure the trigger bindings
    configureBindings();

    m_drivetrain.setDefaultCommand(m_swerveDriveClosedLoop);
    m_climber.setDefaultCommand(m_manualClimb);
    m_endEffector.setDefaultCommand(m_manualEndEffector);
    m_elevator.setDefaultCommand(m_manualElevator);
    m_sourceIntake.setDefaultCommand(m_manualSourceIntake);

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
    Button.leftTrigger1.whileTrue(m_reefAlign);
    Button.rightBumper1.whileTrue(m_coralStationAlign);

    /* Elevator */
    Button.psButton1.onTrue(m_elevatorToStow);

    /* Scoring */
    Button.controlPadDown1.onTrue(m_scoreAlgae);
    Button.controlPadRight1.onTrue(new ConditionalCommand(m_pickupAlgaeDown1, m_pickupAlgaeUp1,
        () -> (ElevatorPositionSelector.getSelectedPosition().height - m_elevator.getPosition()[0] < 0)));
    Button.controlPadUp1.onTrue(new ConditionalCommand(m_pickupAlgaeDown2, m_pickupAlgaeUp2,
        () -> (ElevatorPositionSelector.getSelectedPosition().height - m_elevator.getPosition()[0] < 0)));
    Button.square1.onTrue(new ConditionalCommand(m_moveDownToLevel, m_moveUpToLevel,
        () -> (ElevatorPositionSelector.getSelectedPosition().height - m_elevator.getPosition()[0] < 0)));

    Button.cross1.onTrue(m_indexCoral/*.andThen(new ReverseCoral(m_endEffector))*/);
    Button.circle1.onTrue(m_scoreCoral);

    Button.rightTrigger1.onTrue(m_testEndEffector.withTimeout(2.0));

    // ==================
    // Operator Controls
    // ==================

    /* End Effector */
    Button.triangle2.onTrue(m_bumpAlgae);

    Button.rightTrigger2.whileTrue(m_IntakeAlgae);
    Button.leftTrigger2.whileTrue(m_OutakeAlgae);

    /* Elevator */
    Button.controlPadDown2.onTrue(m_setLevelOne);
    Button.leftBumper2.onTrue(m_setLevelSource);
    Button.controlPadLeft2.onTrue(m_setLevelTwo);
    Button.controlPadRight2.onTrue(m_setLevelThree);
    Button.controlPadUp2.onTrue(m_setLevelFour);
    Button.square2.onTrue(m_elevatorToStow);

    Button.cross2.onTrue(m_goToSourceIntakeAngle1);
    Button.circle2.onTrue(m_goToSourceIntakeAngle2);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // return m_autoSelector.getAutoRoutine();
    return null;
  }

  public Command getTeleopInitCommand() {
    return m_goToSourceIntakeAngle1;
  }

}