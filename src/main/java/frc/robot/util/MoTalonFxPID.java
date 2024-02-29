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
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Velocity;
import frc.robot.encoder.TalonFxEncoder;
import java.util.function.Function;

public class MoTalonFxPID<Dim extends Unit<Dim>> {
    private final Type type;
    private final TalonFX motorController;
    private final Slot0Configs slotPIDConfigs = new Slot0Configs();
    private double lastReference;

    private Dim internalEncoderUnits;
    protected final Velocity<Dim> internalEncoderVelocity;

    public MoTalonFxPID(Type type, TalonFX controller, Dim internalEncoderUnits) {
        this.type = type;
        this.motorController = controller;

        // Load the motor controller's Slot 0 PID values into slotPIDConfigs
        this.motorController.getConfigurator().refresh(slotPIDConfigs);

        this.internalEncoderUnits = internalEncoderUnits;
        this.internalEncoderVelocity = internalEncoderUnits.per(TalonFxEncoder.VELOCITY_BASE_UNIT);
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
                return this.motorController.getPosition().getValueAsDouble();
            case VELOCITY:
                return this.motorController.getVelocity().getValueAsDouble();
        }

        return 0;
    }

    /**
     * Set the reference of the PID controller. The units of value depend on the current type of the controller.
     * For position controllers (Type.POSITION or Type.SMARTMOTION), value is measured in internalEncoderUnits.
     * For velocity controllers (Type.VELOCITY or Type.SMARTVELOCITY), value is measured in internalEncoderUnits per second.
     * <p>
     * @deprecated Use {@link #setPositionReference(Measure)} or {@link #setVelocityReference(Measure)}
     */
    @Deprecated(forRemoval = false)
    public void setReference(double value) {
        this.motorController.setControl(this.type.control.apply(value));
        this.lastReference = value;
    }

    public void setPositionReference(Measure<Dim> position) {
        if (this.type != Type.POSITION && this.type != Type.SMARTMOTION) {
            throw new UnsupportedOperationException(
                    String.format("Cannot set position on PID controller of type %s", this.type.name()));
        }
        double value = position.in(internalEncoderUnits);
        this.motorController.setControl(this.type.control.apply(value));
        lastReference = value;
    }

    public void setVelocityReference(Measure<Velocity<Dim>> velocity) {
        if (this.type != Type.VELOCITY) {
            throw new UnsupportedOperationException(
                    String.format("Cannot set velocity on PID controller of type %s", this.type.name()));
        }

        double value = velocity.in(internalEncoderVelocity);
        this.motorController.setControl(this.type.control.apply(value));
        lastReference = value;
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
