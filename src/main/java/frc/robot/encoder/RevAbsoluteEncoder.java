package frc.robot.encoder;

import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;

public class RevAbsoluteEncoder implements MoEncoder.Encoder {
    public static Time VELOCITY_BASE_UNIT = Units.Seconds;

    private AbsoluteEncoder encoder;

    public RevAbsoluteEncoder(AbsoluteEncoder encoder) {
        this.encoder = encoder;

        encoder.setPositionConversionFactor(1);
        encoder.setVelocityConversionFactor(1);
    }

    @Override
    public double getPosition() {
        return encoder.getPosition();
    }

    @Override
    public void setPosition(double position) {
        throw new UnsupportedOperationException("Cannot set position on an absolute encoder");
    }

    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }

    @Override
    public void setPositionFactor(double factor) {
        encoder.setPositionConversionFactor(factor);
        encoder.setVelocityConversionFactor(factor);
    }

    @Override
    public Time getVelocityBaseUnit() {
        return VELOCITY_BASE_UNIT;
    }

    @Override
    public void setInverted(boolean inverted) {
        encoder.setInverted(inverted);
    }
}
