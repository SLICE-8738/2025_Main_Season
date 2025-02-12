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
        var positionalFXMotorOutput = positionalFXConfiguration.MotorOutput;
        positionalFXMotorOutput.Inverted = Constants.kpositionalFX.positionalFX_INVERT;
        positionalFXMotorOutput.NeutralMode = Constants.kpositionalFX.positionalFX_IDLE;

        /* Current Limiting */
        var positionalFXCurrentLimits = positionalFXConfiguration.CurrentLimits;
        positionalFXCurrentLimits.SupplyCurrentLimitEnable = Constants.kpositionalFX.positionalFX_ENABLE_SUPPLY_CURRENT_LIMIT;
        positionalFXCurrentLimits.SupplyCurrentLimit = Constants.kpositionalFX.positionalFX_SUPPLY_CURRENT_LIMIT;
        positionalFXCurrentLimits.SupplyCurrentLowerLimit = Constants.kpositionalFX.positionalFX_SUPPLY_CURRENT_LOWER_LIMIT;
        positionalFXCurrentLimits.SupplyCurrentLowerTime = Constants.kpositionalFX.positionalFX_SUPPLY_CURRENT_LOWER_TIME;

        positionalFXCurrentLimits.StatorCurrentLimitEnable = Constants.kpositionalFX.positionalFX_ENABLE_STATOR_CURRENT_LIMIT;
        positionalFXCurrentLimits.StatorCurrentLimit = Constants.kpositionalFX.positionalFX_STATOR_CURRENT_LIMIT;

        /* PID */
        var positionalFXSlot0 = positionalFXConfiguration.Slot0;
        positionalFXSlot0.kP = Constants.kpositionalFX.positionalFX_KP;
        positionalFXSlot0.kI = Constants.kpositionalFX.positionalFX_KI;
        positionalFXSlot0.kD = Constants.kpositionalFX.positionalFX_KD;

        /* Open and Closed Loop Ramping */
        positionalFXConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.kpositionalFX.OPEN_LOOP_RAMP;
        positionalFXConfiguration.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.kpositionalFX.OPEN_LOOP_RAMP;

        positionalFXConfiguration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.kpositionalFX.CLOSED_LOOP_RAMP;
        positionalFXConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kpositionalFX.CLOSED_LOOP_RAMP;
    }
}