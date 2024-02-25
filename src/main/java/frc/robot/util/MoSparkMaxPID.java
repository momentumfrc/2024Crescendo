// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import frc.robot.encoder.RevRelativeEncoder;

public class MoSparkMaxPID {
    private final Type type;
    private final CANSparkMax motorController;
    private final SparkPIDController pidController;
    private final RelativeEncoder encoder;
    private final int pidSlot;
    private double lastReference;

    protected final Angle internalEncoderUnits;
    protected final Velocity<Angle> internalEncoderVelocity;

    /**
     * Constructs a MoSparkMaxPID
     * <p>
     * Note: the controller's internal encoder should be scaled to return the mechanism's position in units of rotation.
     *
     * @param type the type of PID controller
     * @param controller the motor controller
     * @param pidSlot the slot in which to save the PID constants
     */
    public MoSparkMaxPID(Type type, CANSparkMax controller, int pidSlot, Angle internalEncoderUnits) {
        this.type = type;
        this.motorController = controller;
        this.pidController = controller.getPIDController();
        this.encoder = controller.getEncoder();
        this.pidSlot = pidSlot;
        this.internalEncoderUnits = internalEncoderUnits;
        this.internalEncoderVelocity = internalEncoderUnits.per(RevRelativeEncoder.VELOCITY_BASE_UNIT);
    }

    public SparkPIDController getPID() {
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

    /**
     * Set the reference of the PID controller. The units of value depend on the current type of the controller.
     * For position controllers (Type.POSITION or Type.SMARTMOTION), value is measured in internalEncoderUnits.
     * For velocity controllers (Type.VELOCITY or Type.SMARTVELOCITY), value is measured in internalEncoderUnits per minute.
     * <p>
     * @deprecated Use {@link #setPositionReference(Measure)} or {@link #setVelocityReference(Measure)}
     */
    @Deprecated(forRemoval = false)
    public void setReference(double value) {
        pidController.setReference(value, this.type.innerType, pidSlot);
        lastReference = value;
    }

    public void setPositionReference(Measure<Angle> position) {
        if (this.type != Type.POSITION && this.type != Type.SMARTMOTION) {
            throw new UnsupportedOperationException(
                    String.format("Cannot set position on PID controller of type %s", this.type.name()));
        }
        double value = position.in(internalEncoderUnits);
        pidController.setReference(value, this.type.innerType, pidSlot);
        lastReference = value;
    }

    public void setVelocityReference(Measure<Velocity<Angle>> velocity) {
        if (this.type != Type.VELOCITY && this.type != Type.SMARTVELOCITY) {
            throw new UnsupportedOperationException(
                    String.format("Cannot set velocity on PID controller of type %s", this.type.name()));
        }
        double value = velocity.in(internalEncoderVelocity);
        pidController.setReference(value, this.type.innerType, pidSlot);
        lastReference = value;
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
