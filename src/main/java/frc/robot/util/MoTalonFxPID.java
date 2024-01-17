// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class MoTalonFxPID {
    /**
     * The talon expects speed measurements in m/100ms, but all our calculations are in m/s. So we
     * just multiply by this conversion factor to adjust.
     */
    private static final double VELOCITY_CONVERSION_FACTOR = 1 / 10.0;

    private final Type type;
    private final TalonFX motorController;
    private double lastReference;

    public MoTalonFxPID(Type type, TalonFX controller) {
        this.type = type;
        this.motorController = controller;
    }

    public Type getType() {
        return type;
    }

    public void setP(double kP) {
        motorController.config_kP(0, kP);
    }

    public void setI(double kI) {
        motorController.config_kI(0, kI);
    }

    public void setD(double kD) {
        motorController.config_kD(0, kD);
    }

    public void setFF(double kFF) {
        motorController.config_kF(0, kFF);
    }

    public void setIZone(double iZone) {
        motorController.config_IntegralZone(0, iZone);
    }

    public double getLastOutput() {
        return this.motorController.getMotorOutputPercent();
    }

    public double getSetpoint() {
        return this.lastReference;
    }

    public double getLastMeasurement() {
        switch (this.type) {
            case POSITION:
            case SMARTMOTION:
                return this.motorController.getSelectedSensorPosition();
            case VELOCITY:
                return this.motorController.getSelectedSensorVelocity();
        }

        return 0;
    }

    public void setReference(double value) {
        if (this.type == Type.VELOCITY) {
            value *= VELOCITY_CONVERSION_FACTOR;
        }
        this.motorController.set(this.type.innerType, value);
        this.lastReference = value;
    }

    public enum Type {
        POSITION(TalonFXControlMode.Position),
        SMARTMOTION(TalonFXControlMode.MotionMagic),
        VELOCITY(TalonFXControlMode.Velocity);

        public final TalonFXControlMode innerType;

        private Type(TalonFXControlMode innerType) {
            this.innerType = innerType;
        }
    }
}
