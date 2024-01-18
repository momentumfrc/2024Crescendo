// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.function.Function;

public class MoTalonFxPID {
    /**
     * The talon expects speed measurements in m/100ms, but all our calculations are in m/s. So we
     * just multiply by this conversion factor to adjust.
     */
    private static final double VELOCITY_CONVERSION_FACTOR = 1 / 10.0;

    private final Type type;
    private final TalonFX motorController;
    private final Slot0Configs slotPIDConfigs = new Slot0Configs();
    private double lastReference;

    public MoTalonFxPID(Type type, TalonFX controller) {
        this.type = type;
        this.motorController = controller;

        // Load the motor controller's Slot 0 PID values into slotPIDConfigs
        this.motorController.getConfigurator().refresh(slotPIDConfigs);
    }

    public Type getType() {
        return type;
    }

    public Slot0Configs getConfigs() {
        return slotPIDConfigs;
    }

    public void applyConfigs() {
        motorController.getConfigurator().apply(slotPIDConfigs);
    }

    public void setP(double kP) {
        slotPIDConfigs.kP = kP;
        applyConfigs();
    }

    public void setI(double kI) {
        slotPIDConfigs.kI = kI;
        applyConfigs();
    }

    public void setD(double kD) {
        slotPIDConfigs.kD = kD;
        applyConfigs();
    }

    public void setFF(double kFF) {
        slotPIDConfigs.kV = kFF;
        applyConfigs();
    }

    public void setIZone(double iZone) {
        // TODO: This appears to no longer be supported by Phoenix v6
    }

    public double getLastOutput() {
        return this.motorController.getBridgeOutput().getValueAsDouble();
    }

    public double getSetpoint() {
        return this.lastReference;
    }

    public double getLastMeasurement() {
        switch (this.type) {
            case POSITION:
            case SMARTMOTION:
                return this.motorController.getRotorPosition().getValueAsDouble();
            case VELOCITY:
                return this.motorController.getRotorVelocity().getValueAsDouble();
        }

        return 0;
    }

    public void setReference(double value) {
        if (this.type == Type.VELOCITY) {
            value *= VELOCITY_CONVERSION_FACTOR;
        }
        this.motorController.setControl(this.type.control.apply(value));
        this.lastReference = value;
    }

    public enum Type {
        POSITION(v -> new PositionVoltage(v)),
        SMARTMOTION(v -> new MotionMagicVoltage(v)),
        VELOCITY(v -> new VelocityVoltage(v));

        public final Function<Double, ControlRequest> control;

        private Type(Function<Double, ControlRequest> control) {
            this.control = control;
        }
    }
}
