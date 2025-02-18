package frc.slicelibs.config;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants;

public final class CTREConfigs {
    public final TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public final TalonFXConfiguration positionalFXConfig = new TalonFXConfiguration();
    public final Pigeon2Configuration pigeon2Config = new Pigeon2Configuration();

    public CTREConfigs() {
        /* Swerve Drive Motor Configuration */

        /* Motor Invert and Neutral Mode */
        var driveMotorOutput = swerveDriveFXConfig.MotorOutput;
        driveMotorOutput.Inverted = Constants.kDrivetrain.DRIVE_INVERT;
        driveMotorOutput.NeutralMode = Constants.kDrivetrain.DRIVE_IDLE_MODE;

        /* Current Limiting */
        var driveCurrentLimits = swerveDriveFXConfig.CurrentLimits;
        driveCurrentLimits.SupplyCurrentLimitEnable = Constants.kDrivetrain.DRIVE_ENABLE_SUPPLY_CURRENT_LIMIT;
        driveCurrentLimits.SupplyCurrentLimit = Constants.kDrivetrain.DRIVE_SUPPLY_CURRENT_LIMIT;
        driveCurrentLimits.SupplyCurrentLowerLimit = Constants.kDrivetrain.DRIVE_SUPPLY_CURRENT_LOWER_LIMIT;
        driveCurrentLimits.SupplyCurrentLowerTime = Constants.kDrivetrain.DRIVE_SUPPLY_CURRENT_LOWER_TIME;

        driveCurrentLimits.StatorCurrentLimitEnable = Constants.kDrivetrain.DRIVE_ENABLE_STATOR_CURRENT_LIMIT;
        driveCurrentLimits.StatorCurrentLimit = Constants.kDrivetrain.DRIVE_STATOR_CURRENT_LIMIT;

        /* PID */
        var driveSlot0 = swerveDriveFXConfig.Slot0;
        driveSlot0.kP = Constants.kDrivetrain.DRIVE_KP;
        driveSlot0.kI = Constants.kDrivetrain.DRIVE_KI;
        driveSlot0.kD = Constants.kDrivetrain.DRIVE_KD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.kDrivetrain.OPEN_LOOP_RAMP;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.kDrivetrain.OPEN_LOOP_RAMP;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.kDrivetrain.CLOSED_LOOP_RAMP;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kDrivetrain.CLOSED_LOOP_RAMP;

        /* Pigeon 2 Gyro Configuration */
        pigeon2Config.GyroTrim.GyroScalarZ = Constants.kDrivetrain.INVERT_GYRO ? -1 : 1;

        /* positionalFX Configuration */

        /* Motor Invert and Neutral Mode */
        var positionalFXMotorOutput = positionalFXConfig.MotorOutput;
        positionalFXMotorOutput.Inverted = Constants.kTalonFXPositionalSubsystem.POSITIONALFX_INVERT;
        positionalFXMotorOutput.NeutralMode = Constants.kTalonFXPositionalSubsystem.POSITIONALFX_IDLE;

        /* Current Limiting */
        var positionalFXCurrentLimits = positionalFXConfig.CurrentLimits;
        positionalFXCurrentLimits.SupplyCurrentLimitEnable = Constants.kTalonFXPositionalSubsystem.POSITIONALFX_ENABLE_SUPPLY_CURRENT_LIMIT;
        positionalFXCurrentLimits.SupplyCurrentLimit = Constants.kTalonFXPositionalSubsystem.POSITIONALFX_SUPPLY_CURRENT_LIMIT;
        positionalFXCurrentLimits.SupplyCurrentLowerLimit = Constants.kTalonFXPositionalSubsystem.POSITIONALFX_SUPPLY_CURRENT_LOWER_LIMIT;
        positionalFXCurrentLimits.SupplyCurrentLowerTime = Constants.kTalonFXPositionalSubsystem.POSITIONALFX_SUPPLY_CURRENT_LOWER_TIME;

        positionalFXCurrentLimits.StatorCurrentLimitEnable = Constants.kTalonFXPositionalSubsystem.POSITIONALFX_ENABLE_STATOR_CURRENT_LIMIT;
        positionalFXCurrentLimits.StatorCurrentLimit = Constants.kTalonFXPositionalSubsystem.POSITIONALFX_STATOR_CURRENT_LIMIT;

        /* PID */
        var positionalFXSlot0 = positionalFXConfig.Slot0;
        positionalFXSlot0.kP = Constants.kTalonFXPositionalSubsystem.POSITIONALFX_KP;
        positionalFXSlot0.kI = Constants.kTalonFXPositionalSubsystem.POSITIONALFX_KI;
        positionalFXSlot0.kD = Constants.kTalonFXPositionalSubsystem.POSITIONALFX_KD;

        /* Open and Closed Loop Ramping */
        positionalFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.kTalonFXPositionalSubsystem.OPEN_LOOP_RAMP;
        positionalFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.kTalonFXPositionalSubsystem.OPEN_LOOP_RAMP;

        positionalFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.kTalonFXPositionalSubsystem.CLOSED_LOOP_RAMP;
        positionalFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kTalonFXPositionalSubsystem.CLOSED_LOOP_RAMP;
    }
}