// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import frc.slicelibs.config.CTREConfigs;
import frc.slicelibs.config.REVConfigs;
import frc.slicelibs.config.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final Mode ADVANTAGE_KIT_MODE = Mode.REAL;
  public static final CTREConfigs CTRE_CONFIGS = new CTREConfigs();
  public static final REVConfigs REV_CONFIGS = new REVConfigs();

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public final class OperatorConstants {

    public static final int DRIVER_CONTROLLER_PORT = 0;

    public static final double DRIVE_EXPONENT = 1.0;
    public static final double DRIVE_EXPONENT_PERCENT = 1;

    public static final double TURN_EXPONENT = 1.0;
    public static final double TURN_EXPONENT_PERCENT = 1;

  }

  public final class kDrivetrain {

    /* Gyro */
    public static final int GYRO_ID = 15;
    public static final boolean INVERT_GYRO = false; // Always ensure Gyro is CCW+ CW-

    /* Swerve Physics */
    public static final double TRACK_WIDTH = Units.inchesToMeters(24.5);
    public static final double WHEEL_BASE = Units.inchesToMeters(24.5);
    public static final double DRIVE_BASE_RADIUS = Math.hypot(WHEEL_BASE / 2, TRACK_WIDTH / 2);
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.95);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    public static final double MASS = 25; // TODO: Find mass (kg)
    public static final double MOMENT_OF_INERTIA = 3; // TODO: Find MOI (kg*m^2)
    public static final double WHEEL_COEFFICIENT_OF_FRICTION = 0.7; // (Vex Griplocks)

    public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0), // Front left module
        new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0), // Front right module
        new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0), // Back right module
        new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0)); // Back left module

    /* Motor Gearing */
    public static final double DRIVE_GEAR_RATIO = (5.14 / 1.0); // 5.14:1
    public static final double ANGLE_GEAR_RATIO = (25.0 / 1.0); // 25:1

    /* Swerve Voltage Compensation */
    public static final double MAX_VOLTAGE = 12.0;

    /* Swerve Current Limiting */
    public static final boolean DRIVE_ENABLE_SUPPLY_CURRENT_LIMIT = true;
    public static final int DRIVE_SUPPLY_CURRENT_LIMIT = 40;
    public static final int DRIVE_SUPPLY_CURRENT_LOWER_LIMIT = 65;
    public static final double DRIVE_SUPPLY_CURRENT_LOWER_TIME = 0.1;

    public static final boolean DRIVE_ENABLE_STATOR_CURRENT_LIMIT = true;
    public static final double DRIVE_STATOR_CURRENT_LIMIT = 65;

    public static final int ANGLE_CURRENT_LIMIT = 20;

    public static final double OPEN_LOOP_RAMP = 0.25;
    public static final double CLOSED_LOOP_RAMP = 0.0;

    /* Status Frame Rates/Periods */
    // TODO: Tune signal frequencies/status frame periods
    public static final int DRIVE_DEFAULT_FREQUENCY_HZ = 22;
    public static final int DRIVE_POSITION_FREQUENCY_HZ = 100;
    public static final int ANGLE_VELOCITY_PERIOD_MS = 1500;
    public static final int ANGLE_POSITION_PERIOD_MS = 300;

    /* Drive Motor PID Values */
    public static final double DRIVE_KP = 0.05; // TODO: Tune drive motor PID gains
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;

    /* Angle Motor PID Values */
    public static final double ANGLE_KP = 0.01;
    public static final double ANGLE_KI = 0.0;
    public static final double ANGLE_KD = 0.002;
    public static final double ANGLE_KFF = 0.0;

    /* Drive Motor Feedforward Values */
    // TODO: Find drive motor feedforward gains from characterization
    public static final double DRIVE_KS = 0.0;
    public static final double DRIVE_KV = 2.1818;
    public static final double DRIVE_KA = 0.01;

    /* Drive Motor Conversion Factors */
    public static final double DRIVE_POSITION_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO;
    public static final double DRIVE_VELOCITY_CONVERSION_FACTOR = DRIVE_POSITION_CONVERSION_FACTOR / 60.0;
    public static final double ANGLE_POSITION_CONVERSION_FACTOR = 360.0 / ANGLE_GEAR_RATIO;
    public static final double ANGLE_VELOCITY_CONVERSION_FACTOR = ANGLE_POSITION_CONVERSION_FACTOR / 60.0;

    /* Swerve Profiling Values */
    public static final double MAX_LINEAR_VELOCITY = 5.5; // meters per second
    public static final double MAX_ANGULAR_VELOCITY = 12.5; // radians per second

    /* PathPlanner Values */
    public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(4.5, 5.5, 10, 12.5);
    public static final double TRANSLATION_KP = 4.5;
    public static final double ROTATION_KP = 1.0;

    /* Motor Idle Modes */
    public static final IdleMode ANGLE_IDLE_MODE = IdleMode.kCoast;
    public static final NeutralModeValue DRIVE_IDLE_MODE = NeutralModeValue.Brake;

    /* Motor Inverts */
    public static final InvertedValue DRIVE_INVERT = InvertedValue.CounterClockwise_Positive;
    public static final boolean ANGLE_INVERT = false;

    /* Absolute Angle Encoder Invert */
    public static final boolean ABSOLUTE_ENCODER_INVERT = false; // TODO: Determine whether to invert

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public final class Mod0 {
      public static final int DRIVE_MOTOR_ID = 4;
      public static final int ANGLE_MOTOR_ID = 8;
      public static final int ABSOLUTE_ENCODER_ID = 2;
      public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(273.3);
      public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(
          DRIVE_MOTOR_ID,
          ANGLE_MOTOR_ID,
          ABSOLUTE_ENCODER_ID,
          ANGLE_OFFSET);
    }

    /* Front Right Module - Module 1 */
    public final class Mod1 {
      public static final int DRIVE_MOTOR_ID = 1;
      public static final int ANGLE_MOTOR_ID = 5;
      public static final int ABSOLUTE_ENCODER_ID = 1;
      public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(295.2);
      public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(
          DRIVE_MOTOR_ID,
          ANGLE_MOTOR_ID,
          ABSOLUTE_ENCODER_ID,
          ANGLE_OFFSET);
    }

    /* Back Right Module - Module 2 */
    public final class Mod2 {
      public static final int DRIVE_MOTOR_ID = 2;
      public static final int ANGLE_MOTOR_ID = 6;
      public static final int ABSOLUTE_ENCODER_ID = 3;
      public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(183.6);
      public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(
          DRIVE_MOTOR_ID,
          ANGLE_MOTOR_ID,
          ABSOLUTE_ENCODER_ID,
          ANGLE_OFFSET);
    }

    /* Back Left Module - Module 3 */
    public final class Mod3 {
      public static final int DRIVE_MOTOR_ID = 3;
      public static final int ANGLE_MOTOR_ID = 7;
      public static final int ABSOLUTE_ENCODER_ID = 0;
      public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(82.1);
      public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(
          DRIVE_MOTOR_ID,
          ANGLE_MOTOR_ID,
          ABSOLUTE_ENCODER_ID,
          ANGLE_OFFSET);
    }

    public static final double LEFT_BRANCH_X_POSITION = -0.1651;
    public static final double RIGHT_BRANCH_X_POSITION = 0.1651;

    public static final double CORAL_STATION_LEFT_X_POSITION = -0.25;
    public static final double CORAL_STATION_RIGHT_X_POSITION = 0.25;

    public static final double ROBOT_FLUSH_SURFACE_Z_POSITION = -0.47;

    public static enum CoralPosition {

      /* Reef Positions */
      BACK_MIDDLE_LEFT_BRANCH(LEFT_BRANCH_X_POSITION, new Pose2d(2.434, 4.055, new Rotation2d()),
          "Back Middle Left Branch"),
      BACK_LEFT_RIGHT_BRANCH(RIGHT_BRANCH_X_POSITION, new Pose2d(3.464, 5.853, Rotation2d.fromDegrees(300)),
          "Back Left Right Branch"),
      BACK_LEFT_LEFT_BRANCH(LEFT_BRANCH_X_POSITION, new Pose2d(3.464, 5.853, Rotation2d.fromDegrees(300)),
          "Back Left Left Branch"),
      FRONT_LEFT_RIGHT_BRANCH(RIGHT_BRANCH_X_POSITION, new Pose2d(5.55, 5.829, Rotation2d.fromDegrees(240)),
          "Front Left Right Branch"),
      FRONT_LEFT_LEFT_BRANCH(LEFT_BRANCH_X_POSITION, new Pose2d(5.55, 5.829, Rotation2d.fromDegrees(240)),
          "Front Left Left Branch"),
      FRONT_MIDDLE_RIGHT_BRANCH(RIGHT_BRANCH_X_POSITION, new Pose2d(6.557, 4.055, Rotation2d.fromDegrees(180)),
          "Front Middle Right Branch"),
      FRONT_MIDDLE_LEFT_BRANCH(LEFT_BRANCH_X_POSITION, new Pose2d(6.557, 4.055, Rotation2d.fromDegrees(180)),
          "Front Middle Left Branch"),
      FRONT_RIGHT_RIGHT_BRANCH(RIGHT_BRANCH_X_POSITION, new Pose2d(5.478, 2.233, Rotation2d.fromDegrees(120)),
          "Front Right Right Branch"),
      FRONT_RIGHT_LEFT_BRANCH(LEFT_BRANCH_X_POSITION, new Pose2d(5.478, 2.233, Rotation2d.fromDegrees(120)),
          "Front Right Left Branch"),
      BACK_RIGHT_RIGHT_BRANCH(RIGHT_BRANCH_X_POSITION, new Pose2d(3.455, 2.201, Rotation2d.fromDegrees(60)),
          "Back Right Right Branch"),
      BACK_RIGHT_LEFT_BRANCH(LEFT_BRANCH_X_POSITION, new Pose2d(3.455, 2.201, Rotation2d.fromDegrees(60)),
          "Back Right Left Branch"),
      BACK_MIDDLE_RIGHT_BRANCH(RIGHT_BRANCH_X_POSITION, new Pose2d(2.434, 4.055, new Rotation2d()),
          "Back Middle Right Branch"),

      /* Coral Station Positions */
      LEFT_CORAL_STATION_RIGHT(CORAL_STATION_RIGHT_X_POSITION, new Pose2d(1.427, 6.764, Rotation2d.fromDegrees(126)),
          "Left Coral Station Right"),
      LEFT_CORAL_STATION_LEFT(CORAL_STATION_LEFT_X_POSITION, new Pose2d(1.427, 6.764, Rotation2d.fromDegrees(126)),
          "Left Coral Station Left"),
      RIGHT_CORAL_STATION_RIGHT(CORAL_STATION_RIGHT_X_POSITION, new Pose2d(1.427, 1.31, Rotation2d.fromDegrees(234)),
          "Right Coral Station Right"),
      RIGHT_CORAL_STATION_LEFT(CORAL_STATION_LEFT_X_POSITION, new Pose2d(1.427, 1.31, Rotation2d.fromDegrees(234)),
          "Right Coral Station Left");

      public final double xAlignPosition;
      public final Pose2d fieldPosition;
      public final String name;

      CoralPosition(double xAlignPosition, Pose2d fieldPosition, String name) {
        this.xAlignPosition = xAlignPosition;
        this.fieldPosition = fieldPosition;
        this.name = name;
      }

    }

  }

  public final class kElevator {

    public static final int LEFT_MOTOR_ID = 9;
    public static final int RIGHT_MOTOR_ID = 10;
    public static final double POSITION_CONVERSION_FACTOR = (0.0382016 * Math.PI) / 3.5; // Pitch diameter times pi (to
                                                                                         // get pitch circumference)
                                                                                         // divided by gear ratio.
    public static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR;
    public static final double KP = 2.85;
    public static final double KI = 0.06;
    public static final double KD = 0.25;
    public static final double THRESHOLD = .01;

    /* Motor Invert */
    public static final InvertedValue ELEVATORFX_INVERT = InvertedValue.Clockwise_Positive;

    /* Motor Idle Modes */
    public static final NeutralModeValue ELEVATORFX_IDLE = NeutralModeValue.Brake;

    /* Current Limiting */
    // TODO: Find current limits
    public static final boolean ELEVATORFX_ENABLE_SUPPLY_CURRENT_LIMIT = true;
    public static final int ELEVATORFX_SUPPLY_CURRENT_LIMIT = 40;
    public static final int ELEVATORFX_SUPPLY_CURRENT_LOWER_LIMIT = 65;
    public static final double ELEVATORFX_SUPPLY_CURRENT_LOWER_TIME = 0.1;

    public static final boolean ELEVATORFX_ENABLE_STATOR_CURRENT_LIMIT = true;
    public static final double ELEVATORFX_STATOR_CURRENT_LIMIT = 65;

    public static final double OPEN_LOOP_RAMP = 0.25;
    public static final double CLOSED_LOOP_RAMP = 0.0;

    /* PID */
    // TODO: Tune PIDs
    public static final double ELEVATORFX_KP = 0.1;
    public static final double ELEVATORFX_KI = 0.001;
    public static final double ELEVATORFX_KD = 0.01;

    /* Elevator Levels */
    public enum Level {
      STOW(0.02, 88),
      LEVEL1(0.254, 45),
      SOURCE(0.2735, 88),
      ALGAE1(0.3, 20),
      LEVEL2(0.51, 85),
      ALGAE2(0.6, 20),
      LEVEL3(0.915, 85),
      LEVEL4(1.625, 55);

      public double height;
      public double angle;

      private Level(double height, double angle) {
        this.height = height;
        this.angle = angle;
      }
    }

  }

  public final class kClimber {

    public static final int MOTOR_ID = 14;
    public static final int ABSOLUTE_ENCODER_ID = 7;

    public static final double ENCODER_OFFSET = 0;

    public static final double POSITIONAL_CONVERSION_FACTOR = 1.0/45.0;
    public static final double VELOCITY_CONVERSION_FACTOR = POSITIONAL_CONVERSION_FACTOR;

    /** The angle of the climber hook, in degrees, when it is fully climbed */
    public static final double CLIMB_POSITION = 140;

    /* Motor Configs */
      public static final InvertedValue CLIMB_INVERT = InvertedValue.CounterClockwise_Positive;
      public static final NeutralModeValue CLIMB_IDLE_MODE = NeutralModeValue.Brake;

      /* Climber Current Limiting */
      public static final boolean CLIMB_ENABLE_SUPPLY_CURRENT_LIMIT = true;
      public static final int CLIMB_SUPPLY_CURRENT_LIMIT = 40;
      public static final int CLIMB_SUPPLY_CURRENT_LOWER_LIMIT = 70;
      public static final double CLIMB_SUPPLY_CURRENT_LOWER_TIME = 0.1;

      public static final boolean CLIMB_ENABLE_STATOR_CURRENT_LIMIT = true;
      public static final double CLIMB_STATOR_CURRENT_LIMIT = 70;

  }

  public final class kLEDs {
    public static final int LED_PWM_PORT = 1;
    public static final int LED_LENGTH = 300;
  }

  public final class kEndEffector {

    public static final int ROTATION_MOTOR_ID = 11;
    public static final int PLACEMENT_MOTOR_ID = 12;

    public static final double POSITIONAL_CONVERSION_FACTOR = 360.0 * (8.0 / 70.0) * (15.0 / 36.0);
    public static final double VELOCITY_CONVERSTION_FACTOR = POSITIONAL_CONVERSION_FACTOR;

    public static final double ENCODER_OFFSET = 296;

  }

  public final class kTalonFXPositionalSubsystem {

    /* Motor Invert */
    public static final InvertedValue POSITIONALFX_INVERT = InvertedValue.Clockwise_Positive;

    /* Motor Idle Modes */
    public static final NeutralModeValue POSITIONALFX_IDLE = NeutralModeValue.Brake;

    /* Current Limiting */
    // TODO: Find current limits
    public static final boolean POSITIONALFX_ENABLE_SUPPLY_CURRENT_LIMIT = true;
    public static final int POSITIONALFX_SUPPLY_CURRENT_LIMIT = 35;
    public static final int POSITIONALFX_SUPPLY_CURRENT_LOWER_LIMIT = 50;
    public static final double POSITIONALFX_SUPPLY_CURRENT_LOWER_TIME = 0.1;
    public static final double DUTYCYCLE_FORWARD_PEAK = 0.5;
    public static final double DUTYCYCLE_REVERSE_PEAK = -0.5;

    public static final boolean POSITIONALFX_ENABLE_STATOR_CURRENT_LIMIT = true;
    public static final double POSITIONALFX_STATOR_CURRENT_LIMIT = 50;

    public static final double OPEN_LOOP_RAMP = 0.25;
    public static final double CLOSED_LOOP_RAMP = 0.0;

    /* PID */
    // TODO: Tune PIDs
    public static final double POSITIONALFX_KP = 0.1;
    public static final double POSITIONALFX_KI = 0.001;
    public static final double POSITIONALFX_KD = 0.01;

  }

}
