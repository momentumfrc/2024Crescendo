package frc.robot.util;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Subclass of a {@link MoSparkMaxPID} that uses an ArmFeedForward to counteract gravity, linearizing the system response.
 */
public class MoSparkMaxArmPID extends MoSparkMaxPID {
    private Optional<ArmFeedforward> armFF = Optional.empty();

    private Supplier<Measure<Angle>> getAngleFromHorizontal;

    private double kS = 0;
    private double kG = 0;
    private double kV = 0;

    private double lastFF;

    private final MutableMeasure<Velocity<Angle>> mutVelocity = MutableMeasure.zero(Units.RotationsPerSecond);

    /**
     * Constructs a MoSparkMaxArmPID.
     * <p>
     * Note: the controller's internal encoder should be scaled to return the arm's position in units of rotation.
     *
     * @param type the type of PID controller
     * @param controller the motor controller
     * @param pidSlot the slot in which to save the PID constants
     * @param getAngleFromHorizontal A supplier that provides the current angle of the arm. This angle should be measured from the
     *     horizontal (i.e. if the provided angle is 0, the arm should be parallel with the floor)
     */
    public MoSparkMaxArmPID(
            Type type, CANSparkMax controller, int pidSlot, Supplier<Measure<Angle>> getAngleFromHorizontal) {
        super(type, controller, pidSlot);
        this.getAngleFromHorizontal = getAngleFromHorizontal;
    }

    public void setKS(double kS) {
        this.kS = 0;
        this.armFF = Optional.empty();
    }

    public void setKG(double kG) {
        this.kG = 0;
        this.armFF = Optional.empty();
    }

    public void setKV(double kV) {
        this.kV = 0;
        this.armFF = Optional.empty();
    }

    public double getLastFF() {
        return lastFF;
    }

    private double getFF(double positionRadians, double velocityRadiansPerSec) {
        if (armFF.isEmpty()) {
            this.armFF = Optional.of(new ArmFeedforward(kS, kG, kV));
        }

        return this.armFF.get().calculate(positionRadians, velocityRadiansPerSec);
    }

    /**
     * Set the reference of the PID controller. The units of value depend on the current type of the controller.
     * For position controllers (Type.POSITION or Type.SMARTMOTION), value is measured in rotations.
     * For velocity controllers (Type.VELOCITY or Type.SMARTVELOCITY), value is measured in rotations per second.
     * <p>
     * @deprecated Use {@link #setPositionReference(Measure)} or {@link #setVelocityReference(Measure)}
     */
    @Deprecated(forRemoval = false)
    @Override
    public void setReference(double value) {
        DriverStation.reportWarning(
                "Recommend using setReference(Measure<Angle>, Measure<Velocity<Angle>>) overload", false);

        double ff;
        if (this.type == Type.POSITION || this.type == Type.SMARTMOTION) {
            ff = getFF(this.getAngleFromHorizontal.get().in(Units.Radians), 0);
        } else {
            ff = getFF(
                    this.getAngleFromHorizontal.get().in(Units.Radians),
                    mutVelocity.mut_replace(value, Units.RotationsPerSecond).in(Units.RadiansPerSecond));
        }

        this.pidController.setReference(value, this.type.innerType, pidSlot, ff);
        this.lastReference = value;
        this.lastFF = ff;
    }

    public void setPositionReference(Measure<Angle> desiredPosition) {
        if (this.type != Type.POSITION && this.type != Type.SMARTMOTION) {
            throw new UnsupportedOperationException(
                    String.format("Cannot set position on PID controller of type %s", this.type.name()));
        }

        double ff = getFF(this.getAngleFromHorizontal.get().in(Units.Radians), 0);
        double value = desiredPosition.in(Units.Rotations);

        this.pidController.setReference(value, this.type.innerType, pidSlot, ff);
        this.lastReference = value;
        this.lastFF = ff;
    }

    public void setVelocityReference(Measure<Velocity<Angle>> desiredVelocity) {
        if (this.type != Type.VELOCITY && this.type != Type.SMARTVELOCITY) {
            throw new UnsupportedOperationException(
                    String.format("Cannot set velocity on PID controller of type %s", this.type.name()));
        }

        double value = desiredVelocity.in(Units.RadiansPerSecond);
        double ff = getFF(this.getAngleFromHorizontal.get().in(Units.Radians), value);

        this.pidController.setReference(value, this.type.innerType, pidSlot, ff);
        this.lastReference = value;
        this.lastFF = ff;
    }
}