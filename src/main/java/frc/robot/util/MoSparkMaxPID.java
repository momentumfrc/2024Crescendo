// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import frc.robot.encoder.MoEncoder;

public class MoSparkMaxPID {
    protected final Type type;
    protected final CANSparkMax motorController;
    protected final SparkPIDController pidController;
    protected final int pidSlot;

    protected final MoEncoder<Angle> internalEncoder;

    protected MutableMeasure<Angle> lastPositionSetpoint = MutableMeasure.zero(Units.Rotations);
    protected MutableMeasure<Velocity<Angle>> lastVelocitySetpoint = MutableMeasure.zero(Units.RotationsPerSecond);

    /**
     * Constructs a MoSparkMaxPID
     * <p>
     * Note: the controller's internal encoder should be scaled to return the mechanism's position in units of rotation.
     * <p>
     * Note: we need the MoEncoder because it is solely responsible for keeping track of the encoder's internal units,
     * which are needed to calculate the units for the setpoints.
     *
     * @param type the type of PID controller
     * @param controller the motor controller
     * @param pidSlot the slot in which to save the PID constants
     */
    public MoSparkMaxPID(Type type, CANSparkMax controller, int pidSlot, MoEncoder<Angle> internalEncoder) {
        this.type = type;
        this.motorController = controller;
        this.pidController = controller.getPIDController();
        this.pidSlot = pidSlot;
        this.internalEncoder = internalEncoder;
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
        switch (this.type) {
            case POSITION:
            case SMARTMOTION:
                return lastPositionSetpoint.in(internalEncoder.getInternalEncoderUnits());
            case VELOCITY:
            case SMARTVELOCITY:
                return lastVelocitySetpoint.in(internalEncoder.getInternalEncoderUnitsPerSec());
        }

        return 0;
    }

    public double getLastMeasurement() {
        switch (this.type) {
            case POSITION:
            case SMARTMOTION:
                return internalEncoder.getPosition().in(internalEncoder.getInternalEncoderUnits());
            case VELOCITY:
            case SMARTVELOCITY:
                return internalEncoder.getVelocity().in(internalEncoder.getInternalEncoderUnitsPerSec());
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
        switch (this.type) {
            case POSITION:
            case SMARTMOTION:
                lastPositionSetpoint.mut_replace(value, internalEncoder.getInternalEncoderUnits());
                break;
            case VELOCITY:
            case SMARTVELOCITY:
                lastVelocitySetpoint.mut_replace(value, internalEncoder.getInternalEncoderUnitsPerSec());
                break;
        }
    }

    public void setPositionReference(Measure<Angle> position) {
        if (this.type != Type.POSITION && this.type != Type.SMARTMOTION) {
            throw new UnsupportedOperationException(
                    String.format("Cannot set position on PID controller of type %s", this.type.name()));
        }
        double value = internalEncoder.positionInEncoderUnits(position);
        pidController.setReference(value, this.type.innerType, pidSlot);
        lastPositionSetpoint.mut_replace(position);
    }

    public void setVelocityReference(Measure<Velocity<Angle>> velocity) {
        if (this.type != Type.VELOCITY && this.type != Type.SMARTVELOCITY) {
            throw new UnsupportedOperationException(
                    String.format("Cannot set velocity on PID controller of type %s", this.type.name()));
        }
        double value = internalEncoder.velocityInEncoderUnits(velocity);
        pidController.setReference(value, this.type.innerType, pidSlot);
        lastVelocitySetpoint.mut_replace(velocity);
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
