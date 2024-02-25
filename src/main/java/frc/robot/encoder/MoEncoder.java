package frc.robot.encoder;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAnalogSensor;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Per;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import frc.robot.util.MoUnits;

/**
 * Wraps an encoder, keeping track of the encoder's internal units.
 */
public class MoEncoder<Dim extends Unit<Dim>> {
    public static interface Encoder {
        public double getPosition();

        public void setPosition(double position);

        public double getVelocity();

        public void setPositionFactor(double factor);

        public void setVelocityFactor(double factor);

        public Time getVelocityBaseUnit();
    }

    private final Encoder encoder;

    /**
     * The units of the value returned by encoder.getPosition(), assuming the encoder has internally applied the set
     * position factor.
     */
    private final Dim internalEncoderUnits;

    private final Velocity<Dim> interEncoderUnitsPerSecond;

    private final MutableMeasure<Dim> mut_pos;
    private final MutableMeasure<Velocity<Dim>> mut_vel;

    private MoEncoder(Encoder encoder, Dim internalEncoderUnits) {
        this.encoder = encoder;
        this.internalEncoderUnits = internalEncoderUnits;
        this.interEncoderUnitsPerSecond = internalEncoderUnits.per(Units.Second);

        mut_pos = MutableMeasure.zero(internalEncoderUnits);
        mut_vel = MutableMeasure.zero(internalEncoderUnits.per(Units.Second));
    }

    public Dim getInternalEncoderUnits() {
        return internalEncoderUnits;
    }

    public Encoder getEncoder() {
        return encoder;
    }

    public void setConversionFactor(Measure<Per<MoUnits.EncoderAngle, Dim>> mechanismConversionFactor) {
        /*
         * We need to set the position factor such that, when we call getPosition(), the value returned is in the desired
         * units as specified by internalEncoderUnits. We can model the internal encoder calculation as such:
         * [ p encoder_ticks ] * [pos_factor internalEncoderUnits / encoder_ticks] = [p internalEncoderUnits]
         * So, to get the output of the encoder in units of internalEncoderUnits, we need to calculate the pos_factor
         * in units of internalEncoderUnits per encoder_ticks, then set that as the positionConversionFactor.
         */
        double positionFactor = 1 / mechanismConversionFactor.in(MoUnits.EncoderTicks.per(internalEncoderUnits));

        /*
         * We need to set the velocity factor such that, when we call getVelocity(), the value returned is in the units
         * of internalEncoderUnits per second. This is complicated by the fact that different encoders use different
         * base time units for their returned velocities. For example, Rev's RelativeEncoder has a base unit of
         * rotations per minute, while CTRE's TalonFX has a base unit of rotations per second. So to get
         * internalEncoderUnits per second, we would need to divide the positionFactor by 60 for Rev, but we could
         * leave it as-is for CTRE.
         *
         * We can model the internal encoder velocity calculation as such:
         * [v encoder_ticks / base_time] * [vel_factor (internalEncoderUnits / encoder_ticks) / (sec / base_time)]
         *     = [v internalEncoderUnits / sec]
         * So, to get the output of the encoder in units of internalEncoderUnits per second, we need to calculate the
         * vel_factor in units of (internalEncoderUnits per encoder_ticks) per (seconds per base_time) and set that
         * as the velocityConversionFactor.
         */
        // There are velocityBaseUnitFactor seconds per base_time
        double velocityBaseUnitFactor = encoder.getVelocityBaseUnit().one().in(Units.Seconds);
        double velocityFactor = positionFactor / velocityBaseUnitFactor;

        encoder.setPositionFactor(positionFactor);
        encoder.setVelocityFactor(velocityFactor);
    }

    public Measure<Dim> getPosition() {
        return mut_pos.mut_replace(encoder.getPosition(), internalEncoderUnits);
    }

    public void setPosition(Measure<Dim> position) {
        encoder.setPosition(position.in(internalEncoderUnits));
    }

    public Measure<Velocity<Dim>> getVelocity() {
        return mut_vel.mut_replace(encoder.getVelocity(), interEncoderUnitsPerSecond);
    }

    public static <Dim extends Unit<Dim>> MoEncoder<Dim> forSparkRelative(
            RelativeEncoder encoder, Dim internalEncoderUnits) {
        return new MoEncoder<Dim>(new RevRelativeEncoder(encoder), internalEncoderUnits);
    }

    public static <Dim extends Unit<Dim>> MoEncoder<Dim> forSparkAbsolute(
            AbsoluteEncoder encoder, Dim internalEncoderUnits) {
        return new MoEncoder<Dim>(new RevAbsoluteEncoder(encoder), internalEncoderUnits);
    }

    public static <Dim extends Unit<Dim>> MoEncoder<Dim> forSparkAnalog(
            SparkAnalogSensor sensor, Dim internalEncoderUnits) {
        return new MoEncoder<Dim>(new RevAnalogSensorEncoder(sensor), internalEncoderUnits);
    }

    public static <Dim extends Unit<Dim>> MoEncoder<Dim> forTalonFx(TalonFX talon, Dim internalEncoderUnits) {
        return new MoEncoder<Dim>(new TalonFxEncoder(talon), internalEncoderUnits);
    }
}
