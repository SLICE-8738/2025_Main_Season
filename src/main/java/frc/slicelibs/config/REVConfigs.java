package frc.slicelibs.config;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;

public class REVConfigs {

    public final SparkMaxConfig defaultVelocitySparkMaxConfig = new SparkMaxConfig();
    public final SparkMaxConfig defaultPositionSparkMaxConfig = new SparkMaxConfig();
    public final SparkMaxConfig driveSparkMaxConfig = new SparkMaxConfig();
    public final SparkMaxConfig angleSparkMaxConfig = new SparkMaxConfig();

    public REVConfigs() {
        /* Default Velocity Motor Configuration */

            /* Motor Invert and Idle Mode */
            defaultVelocitySparkMaxConfig.inverted(false);
            defaultVelocitySparkMaxConfig.idleMode(IdleMode.kBrake);

            /* Current Limiting */
            defaultVelocitySparkMaxConfig.smartCurrentLimit(30);

            /* Open and Closed Loop Ramping */
            defaultVelocitySparkMaxConfig.openLoopRampRate(0);
            defaultVelocitySparkMaxConfig.closedLoopRampRate(0);

            /* Status Frame Periods */
            defaultVelocitySparkMaxConfig.signals.primaryEncoderVelocityPeriodMs(200);
            defaultVelocitySparkMaxConfig.signals.primaryEncoderPositionPeriodMs(1000);

            /* Voltage Compensation */
            defaultVelocitySparkMaxConfig.voltageCompensation(12);

        /* Default Position Motor Configuration */

            /* Motor Invert and Idle Mode */
            defaultPositionSparkMaxConfig.inverted(false);
            defaultPositionSparkMaxConfig.idleMode(IdleMode.kBrake);

            /* Current Limiting */
            defaultPositionSparkMaxConfig.smartCurrentLimit(20);

            /* Open and Closed Loop Ramping */
            defaultPositionSparkMaxConfig.openLoopRampRate(0);
            defaultPositionSparkMaxConfig.closedLoopRampRate(0);

            /* Status Frame Periods */
            defaultPositionSparkMaxConfig.signals.primaryEncoderVelocityPeriodMs(1500);
            defaultPositionSparkMaxConfig.signals.primaryEncoderPositionPeriodMs(300);

            /* Voltage Compensation */
            defaultPositionSparkMaxConfig.voltageCompensation(12);

        /* Swerve Module Drive Motor Configuration */

            /* Motor Invert and Idle Mode */
            driveSparkMaxConfig.inverted(Constants.kDrivetrain.DRIVE_INVERT);
            driveSparkMaxConfig.idleMode(Constants.kDrivetrain.DRIVE_IDLE_MODE);

            /* Current Limiting */
            driveSparkMaxConfig.smartCurrentLimit(Constants.kDrivetrain.DRIVE_CURRENT_LIMIT);

            /* Open and Closed Loop Ramping */
            driveSparkMaxConfig.openLoopRampRate(Constants.kDrivetrain.OPEN_LOOP_RAMP);
            driveSparkMaxConfig.closedLoopRampRate(Constants.kDrivetrain.CLOSED_LOOP_RAMP);

            /* Status Frame Periods */
            driveSparkMaxConfig.signals.primaryEncoderVelocityPeriodMs(Constants.kDrivetrain.DRIVE_VELOCITY_PERIOD_MS);
            driveSparkMaxConfig.signals.primaryEncoderPositionPeriodMs(Constants.kDrivetrain.DRIVE_POSITION_PERIOD_MS);

            /* Voltage Compensation */
            driveSparkMaxConfig.voltageCompensation(12);

            /* Conversion Factors */
            driveSparkMaxConfig.encoder.positionConversionFactor(Constants.kDrivetrain.DRIVE_POSITION_CONVERSION_FACTOR);
            driveSparkMaxConfig.encoder.velocityConversionFactor(Constants.kDrivetrain.DRIVE_VELOCITY_CONVERSION_FACTOR);

            /* PID */
            driveSparkMaxConfig.closedLoop.p(Constants.kDrivetrain.DRIVE_KP);
            driveSparkMaxConfig.closedLoop.i(Constants.kDrivetrain.DRIVE_KI);
            driveSparkMaxConfig.closedLoop.d(Constants.kDrivetrain.DRIVE_KD);

        /* Swerve Module Angle Motor Configuration */

            /* Motor Invert and Idle Mode */
            angleSparkMaxConfig.inverted(Constants.kDrivetrain.ANGLE_INVERT);
            angleSparkMaxConfig.idleMode(Constants.kDrivetrain.ANGLE_IDLE_MODE);

            /* Current Limiting */
            angleSparkMaxConfig.smartCurrentLimit(Constants.kDrivetrain.ANGLE_CURRENT_LIMIT);

            /* Open and Closed Loop Ramping */
            angleSparkMaxConfig.openLoopRampRate(0);
            angleSparkMaxConfig.closedLoopRampRate(0);

            /* Status Frame Periods */
            angleSparkMaxConfig.signals.primaryEncoderVelocityPeriodMs(Constants.kDrivetrain.ANGLE_VELOCITY_PERIOD_MS);
            angleSparkMaxConfig.signals.primaryEncoderPositionPeriodMs(Constants.kDrivetrain.ANGLE_POSITION_PERIOD_MS);

            /* Voltage Compensation */
            angleSparkMaxConfig.voltageCompensation(12);

            /* Conversion Factors */
            angleSparkMaxConfig.encoder.positionConversionFactor(Constants.kDrivetrain.ANGLE_POSITION_CONVERSION_FACTOR);
            angleSparkMaxConfig.encoder.velocityConversionFactor(Constants.kDrivetrain.ANGLE_VELOCITY_CONVERSION_FACTOR);

            /* PID */
            angleSparkMaxConfig.closedLoop.p(Constants.kDrivetrain.ANGLE_KP);
            angleSparkMaxConfig.closedLoop.i(Constants.kDrivetrain.ANGLE_KI);
            angleSparkMaxConfig.closedLoop.d(Constants.kDrivetrain.ANGLE_KD);
    }

}
