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

        public void setInverted(boolean inverted);

        /**
         * Set the factor used internally by the encoder to scale encoder ticks to the desired output units.
         * <p>
         * Note: the factor is in units of internalEncoderUnits / encoder_tick.
         * @param factor The factor to be internally multiplied by encoder ticks.
         */
        public void setPositionFactor(double factor);

        /**
         * Velocity is measured in distance per time. However, the dividing time unit is different across different
         * encoders. For example, CTRE returns velocity in units of internalEncoderUnits per *second*, but
         * Rev returns velocity in units of internalEncoderUnits per *minute*. So, for a CTRE encoder this method
         * would return Units.Second, but for a Rev encoder this method would return Units.Minute.
         * @return The base time unit of the velocity returned by this encoder.
         */
        public Time getVelocityBaseUnit();
    }

    private final Encoder encoder;

    /**
     * The units of the value returned by encoder.getPosition(), assuming the encoder has internally applied the set
     * position factor.
     */
    private final Dim internalEncoderUnits;

    /*
     * The units of the value returned by encoder.getVelocity(), assuming the encoder has internally applied the set
     * position factor.
     */
    private final Velocity<Dim> internalEncoderVelocityUnits;

    private final MutableMeasure<Dim> mut_pos;
    private final MutableMeasure<Velocity<Dim>> mut_vel;

    private MoEncoder(Encoder encoder, Dim internalEncoderUnits) {
        this.encoder = encoder;
        this.internalEncoderUnits = internalEncoderUnits;
        this.internalEncoderVelocityUnits = internalEncoderUnits.per(encoder.getVelocityBaseUnit());

        mut_pos = MutableMeasure.zero(internalEncoderUnits);
        mut_vel = MutableMeasure.zero(internalEncoderVelocityUnits);
    }

    public Dim getInternalEncoderUnits() {
        return internalEncoderUnits;
    }

    public Velocity<Dim> getInternalEncoderVelocityUnits() {
        return internalEncoderVelocityUnits;
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

        encoder.setPositionFactor(positionFactor);
    }

    public Measure<Dim> getPosition() {
        return mut_pos.mut_replace(encoder.getPosition(), internalEncoderUnits);
    }

    public void setPosition(Measure<Dim> position) {
        encoder.setPosition(position.in(internalEncoderUnits));
    }

    public Measure<Velocity<Dim>> getVelocity() {
        return mut_vel.mut_replace(encoder.getVelocity(), internalEncoderVelocityUnits);
    }

    public double getPositionInEncoderUnits() {
        return getPosition().in(internalEncoderUnits);
    }

    public double getVelocityInEncoderUnits() {
        return getVelocity().in(internalEncoderVelocityUnits);
    }

    public void setInverted(boolean inverted) {
        encoder.setInverted(inverted);
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
