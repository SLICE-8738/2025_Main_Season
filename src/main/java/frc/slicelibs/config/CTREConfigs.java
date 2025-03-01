package frc.slicelibs.config;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants;

public final class CTREConfigs {

    public final TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public final Pigeon2Configuration pigeon2Config = new Pigeon2Configuration();
    public final TalonFXConfiguration positionalFXConfig = new TalonFXConfiguration();
    public final TalonFXConfiguration climberFXConfig = new TalonFXConfiguration();
    public final TalonFXConfiguration elevatorFXConfig = new TalonFXConfiguration();
    public final TalonFXConfiguration sourceIntakeFXConfig = new TalonFXConfiguration();

    public CTREConfigs() {
        /* ================================ */
        /* Swerve Drive Motor Configuration */
        /* ================================ */

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

        /* ====================================== */
        /* Default Positional Motor Configuration */
        /* ====================================== */

            /* Motor Invert and Neutral Mode */
            var positionalFXMotorOutput = positionalFXConfig.MotorOutput;
            positionalFXMotorOutput.Inverted = Constants.kTalonFXPositionalSubsystem.POSITIONALFX_INVERT;
            positionalFXMotorOutput.NeutralMode = Constants.kTalonFXPositionalSubsystem.POSITIONALFX_IDLE;

            positionalFXConfig.Voltage.PeakForwardVoltage = Constants.kTalonFXPositionalSubsystem.VOLTAGE_FORWARD_PEAK;
            positionalFXConfig.Voltage.PeakReverseVoltage = Constants.kTalonFXPositionalSubsystem.VOLTAGE_REVERSE_PEAK;

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

        /* =========================== */
        /* Climber Motor Configuration */
        /* =========================== */

            /* Motor Invert and Neutral Mode */
            var climbMotorOutput = climberFXConfig.MotorOutput;
            climbMotorOutput.Inverted = Constants.kClimber.CLIMB_INVERT;
            climbMotorOutput.NeutralMode = Constants.kClimber.CLIMB_IDLE_MODE;

            /* Current Limiting */
            var climbCurrentLimits = climberFXConfig.CurrentLimits;
            climbCurrentLimits.SupplyCurrentLimitEnable = Constants.kClimber.CLIMB_ENABLE_SUPPLY_CURRENT_LIMIT;
            climbCurrentLimits.SupplyCurrentLimit = Constants.kClimber.CLIMB_SUPPLY_CURRENT_LIMIT;
            climbCurrentLimits.SupplyCurrentLowerLimit = Constants.kClimber.CLIMB_SUPPLY_CURRENT_LOWER_LIMIT;
            climbCurrentLimits.SupplyCurrentLowerTime = Constants.kClimber.CLIMB_SUPPLY_CURRENT_LOWER_TIME;

            climbCurrentLimits.StatorCurrentLimitEnable = Constants.kClimber.CLIMB_ENABLE_STATOR_CURRENT_LIMIT;
            climbCurrentLimits.StatorCurrentLimit = Constants.kClimber.CLIMB_STATOR_CURRENT_LIMIT;

        /* ============================ */
        /* Elevator Motor Configuration */
        /* ============================ */

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

        /* ================================ */
        /* Source Intake Motor Configuration */
        /* ================================ */

        /* Motor Invert and Neutral Mode */
        var sourceIntakeMotorOutput = sourceIntakeFXConfig.MotorOutput;
        sourceIntakeMotorOutput.Inverted = Constants.kSourceIntake.MOTOR_INVERT;
        sourceIntakeMotorOutput.NeutralMode = Constants.kSourceIntake.MOTOR_IDLE_MODE;

        /* Current Limiting */
        var sourceIntakeCurrentLimits = sourceIntakeFXConfig.CurrentLimits;
        sourceIntakeCurrentLimits.SupplyCurrentLimitEnable = Constants.kSourceIntake.ENABLE_SUPPLY_CURRENT_LIMIT;
        sourceIntakeCurrentLimits.SupplyCurrentLimit = Constants.kSourceIntake.SUPPLY_CURRENT_LIMIT;
        sourceIntakeCurrentLimits.SupplyCurrentLowerLimit = Constants.kSourceIntake.SUPPLY_CURRENT_LOWER_LIMIT;
        sourceIntakeCurrentLimits.SupplyCurrentLowerTime = Constants.kSourceIntake.SUPPLY_CURRENT_LOWER_TIME;

        sourceIntakeCurrentLimits.StatorCurrentLimitEnable = Constants.kSourceIntake.ENABLE_STATOR_CURRENT_LIMIT;
        sourceIntakeCurrentLimits.StatorCurrentLimit = Constants.kSourceIntake.STATOR_CURRENT_LIMIT;

        var sourceIntakeFXSlot0 = sourceIntakeFXConfig.Slot0;
        sourceIntakeFXSlot0.kP = Constants.kSourceIntake.KP;
        sourceIntakeFXSlot0.kI = Constants.kSourceIntake.KI;
        sourceIntakeFXSlot0.kD = Constants.kSourceIntake.KD;

    }
    
}
