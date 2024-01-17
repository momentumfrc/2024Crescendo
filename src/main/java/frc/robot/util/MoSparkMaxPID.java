// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class MoSparkMaxPID {
    private final Type type;
    private final CANSparkMax motorController;
    private final SparkMaxPIDController pidController;
    private final RelativeEncoder encoder;
    private final int pidSlot;
    private double lastReference;

    public MoSparkMaxPID(Type type, CANSparkMax controller, int pidSlot) {
        this.type = type;
        this.motorController = controller;
        this.pidController = controller.getPIDController();
        this.encoder = controller.getEncoder();
        this.pidSlot = pidSlot;
    }

    public SparkMaxPIDController getPID() {
        return pidController;
    }

    public Type getType() {
        return type;
    }

    public int getPidSlot() {
        return pidSlot;
    }

    public void setP(double kP) {
        pidController.setP(kP, pidSlot);
    }

    public void setI(double kI) {
        pidController.setI(kI, pidSlot);
    }

    public void setD(double kD) {
        pidController.setD(kD, pidSlot);
    }

    public void setFF(double kFF) {
        pidController.setFF(kFF, pidSlot);
    }

    public void setIZone(double iZone) {
        pidController.setIZone(iZone, pidSlot);
    }

    public double getLastOutput() {
        return this.motorController.get();
    }

    public double getSetpoint() {
        return this.lastReference;
    }

    public double getLastMeasurement() {
        switch (this.type) {
            case POSITION:
            case SMARTMOTION:
                return this.encoder.getPosition();
            case VELOCITY:
            case SMARTVELOCITY:
                return this.encoder.getVelocity();
        }

        return 0;
    }

    public void setReference(double value) {
        this.getPID().setReference(value, this.type.innerType, pidSlot);
        this.lastReference = value;
    }

    public enum Type {
        POSITION(CANSparkMax.ControlType.kPosition),
        SMARTMOTION(CANSparkMax.ControlType.kSmartMotion),
        VELOCITY(CANSparkMax.ControlType.kVelocity),
        SMARTVELOCITY(CANSparkMax.ControlType.kSmartVelocity);

        public final CANSparkMax.ControlType innerType;

        private Type(CANSparkMax.ControlType innerType) {
            this.innerType = innerType;
        }
    }
}
