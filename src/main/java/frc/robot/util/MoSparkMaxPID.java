// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Velocity;
import frc.robot.encoder.MoEncoder;

public class MoSparkMaxPID<Dim extends Unit<Dim>> {
    protected final Type type;
    protected final CANSparkBase motorController;
    protected final SparkPIDController pidController;
    protected final int pidSlot;

    protected final MoEncoder<Dim> internalEncoder;

    /**
     * The last position or velocity setpoint measured in internal encoder units.
     */
    protected double lastSetpoint;

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
    public MoSparkMaxPID(Type type, CANSparkBase controller, int pidSlot, MoEncoder<Dim> internalEncoder) {
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

    /**
     * Get the last position or velocity reference set on this PID controller, measured in internal encoder units.
     * @return The last setpoint of this controller
     */
    public double getSetpoint() {
        return lastSetpoint;
    }

    /**
     * Get the last position or velocity measurement of the mechanism controlled by this PID controller, measured in
     * internal encoder units.
     * @return The last measurement of the mechanism under control
     */
    public double getLastMeasurement() {
        switch (this.type) {
            case POSITION:
            case SMARTMOTION:
                return internalEncoder.getPositionInEncoderUnits();
            case VELOCITY:
            case SMARTVELOCITY:
                return internalEncoder.getVelocityInEncoderUnits();
        }

        return 0;
    }

    /**
     * Set the reference of the PID controller. The units of value depend on the current type of the controller.
     * For position controllers (Type.POSITION or Type.SMARTMOTION), value is measured in internalEncoderUnits.
     * For velocity controllers (Type.VELOCITY or Type.SMARTVELOCITY), value is measured in internalEncoderVelocityUnits.
     * <p>
     * @deprecated Use {@link #setPositionReference(Measure)} or {@link #setVelocityReference(Measure)}
     */
    @Deprecated(forRemoval = false)
    public void setReference(double value) {
        pidController.setReference(value, this.type.innerType, pidSlot);
        this.lastSetpoint = value;
    }

    public void setPositionReference(Measure<Dim> position) {
        if (this.type != Type.POSITION && this.type != Type.SMARTMOTION) {
            throw new UnsupportedOperationException(
                    String.format("Cannot set position on PID controller of type %s", this.type.name()));
        }
        double value = position.in(internalEncoder.getInternalEncoderUnits());
        pidController.setReference(value, this.type.innerType, pidSlot);
        lastSetpoint = value;
    }

    public void setVelocityReference(Measure<Velocity<Dim>> velocity) {
        if (this.type != Type.VELOCITY && this.type != Type.SMARTVELOCITY) {
            throw new UnsupportedOperationException(
                    String.format("Cannot set velocity on PID controller of type %s", this.type.name()));
        }
        double value = velocity.in(internalEncoder.getInternalEncoderVelocityUnits());
        pidController.setReference(value, this.type.innerType, pidSlot);
        lastSetpoint = value;
    }

    public enum Type {
        POSITION(CANSparkBase.ControlType.kPosition),
        SMARTMOTION(CANSparkBase.ControlType.kSmartMotion),
        VELOCITY(CANSparkBase.ControlType.kVelocity),
        SMARTVELOCITY(CANSparkBase.ControlType.kSmartVelocity);

        public final CANSparkBase.ControlType innerType;

        private Type(CANSparkBase.ControlType innerType) {
            this.innerType = innerType;
        }
    }
}
