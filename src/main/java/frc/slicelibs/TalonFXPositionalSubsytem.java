// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.slicelibs;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class TalonFXPositionalSubsytem extends SubsystemBase {
    private TalonFX[] motors;
    private double positionConversionFactor;
    private double velocityConversionFactor;
    private double positionTargetReference;
    private double velocityTargetReference;

    public TalonFXPositionalSubsytem(int[] ids, boolean[] inverted, double kP, double kI, double kD,
            GravityTypeValue gravityType, double positionConversionFactor, double velocityConversionFactor) {
        if (ids.length != inverted.length)
            throw new IllegalArgumentException("ids and inverted must be the same length");

        motors = new TalonFX[ids.length];
        for (int i = 0; i < ids.length; i++) {
            TalonFXConfiguration configs = Constants.CTRE_CONFIGS.positionalFXConfig;

            if (inverted[i]) {
                configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            }
            else{
                configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            }
            configs.Slot0.kP = kP;
            configs.Slot0.kI = kI;
            configs.Slot0.kD = kD;
            configs.Slot0.GravityType = gravityType;

            motors[i] = new TalonFX(ids[i]);
            motors[i].getConfigurator().apply(configs);
        }

        this.positionConversionFactor = positionConversionFactor;
        this.velocityConversionFactor = velocityConversionFactor;
        positionTargetReference = 0;
        velocityTargetReference = 0;
    }

    public void set(double speed) {
        for (TalonFX motor : motors) {
            motor.set(speed);
        }
    }

    public void setVelocity(double velocity) {
        VelocityVoltage request = new VelocityVoltage(0).withSlot(0);

        for (TalonFX motor : motors) {
            motor.setControl(request.withVelocity(velocity / velocityConversionFactor));
        }
        velocityTargetReference = velocity;
    }

    public void setPosition(double position) {
        PositionVoltage request = new PositionVoltage(0).withSlot(0);
        for (TalonFX motor : motors) {
            motor.setControl(request.withPosition(position / positionConversionFactor));
        }
        positionTargetReference = position;
        SmartDashboard.putNumber("Elevator Target", position);
    }

    public void setEncoderPosition(double position) {
        for (TalonFX motor : motors) {
            motor.getConfigurator().setPosition(position / positionConversionFactor);
        }
    }

    public double[] getVelocity() {
        double[] velocity = new double[motors.length];
        for (int i = 0; i < motors.length; i++) {
            velocity[i] = motors[i].getVelocity().getValueAsDouble() * velocityConversionFactor;
        }
        return velocity;
    }

    public double[] getPosition() {
        double[] position = new double[motors.length];
        for (int i = 0; i < motors.length; i++) {
            position[i] = motors[i].getPosition().getValueAsDouble() * positionConversionFactor;
        }
        return position;
    }

    public boolean atTarget(double threshold) {
        boolean[] atTarget = new boolean[motors.length];
        if (motors[0].getAppliedControl().getClass() == PositionVoltage.class) {
            for (int i = 0; i < motors.length; i++) {
                atTarget[i] = Math
                        .abs(motors[i].getPosition().getValueAsDouble() - (positionTargetReference / positionConversionFactor)) < (threshold / positionConversionFactor);
            }
        } else if (motors[0].getAppliedControl().getClass() == VelocityVoltage.class) {
            for (int i = 0; i < motors.length; i++) {
                atTarget[i] = Math
                        .abs(motors[i].getVelocity().getValueAsDouble() - (velocityTargetReference / velocityConversionFactor)) < (threshold / velocityConversionFactor);
            }
        }

        for (boolean target : atTarget) {
            if (!target) {
                return false;
            }
        }
        return true;
    }
}