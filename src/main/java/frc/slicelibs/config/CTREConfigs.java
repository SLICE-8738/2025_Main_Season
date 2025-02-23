package frc.slicelibs.config;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants;

public final class CTREConfigs {
    public final TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public final TalonFXConfiguration elevatorFXConfig = new TalonFXConfiguration();
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

        /* elevatorFX Configuration */

        /* Motor Invert and Neutral Mode */
        var elevatorFXMotorOutput = elevatorFXConfig.MotorOutput;
        elevatorFXMotorOutput.Inverted = Constants.kElevator.ELEVATORFX_INVERT;
        elevatorFXMotorOutput.NeutralMode = Constants.kElevator.ELEVATORFX_IDLE;

        /* Current Limiting */
        var elevatorFXCurrentLimits = elevatorFXConfig.CurrentLimits;
        elevatorFXCurrentLimits.SupplyCurrentLimitEnable = Constants.kElevator.ELEVATORFX_ENABLE_SUPPLY_CURRENT_LIMIT;
        elevatorFXCurrentLimits.SupplyCurrentLimit = Constants.kElevator.ELEVATORFX_SUPPLY_CURRENT_LIMIT;
        elevatorFXCurrentLimits.SupplyCurrentLowerLimit = Constants.kElevator.ELEVATORFX_SUPPLY_CURRENT_LOWER_LIMIT;
        elevatorFXCurrentLimits.SupplyCurrentLowerTime = Constants.kElevator.ELEVATORFX_SUPPLY_CURRENT_LOWER_TIME;

        elevatorFXCurrentLimits.StatorCurrentLimitEnable = Constants.kElevator.ELEVATORFX_ENABLE_STATOR_CURRENT_LIMIT;
        elevatorFXCurrentLimits.StatorCurrentLimit = Constants.kElevator.ELEVATORFX_STATOR_CURRENT_LIMIT;

        /* PID */
        var elevatorFXSlot0 = elevatorFXConfig.Slot0;
        elevatorFXSlot0.kP = Constants.kElevator.ELEVATORFX_KP;
        elevatorFXSlot0.kI = Constants.kElevator.ELEVATORFX_KI;
        elevatorFXSlot0.kD = Constants.kElevator.ELEVATORFX_KD;

        /* Open and Closed Loop Ramping */
        elevatorFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.kElevator.OPEN_LOOP_RAMP;
        elevatorFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.kElevator.OPEN_LOOP_RAMP;

        elevatorFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.kElevator.CLOSED_LOOP_RAMP;
        elevatorFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kElevator.CLOSED_LOOP_RAMP;
    }
}