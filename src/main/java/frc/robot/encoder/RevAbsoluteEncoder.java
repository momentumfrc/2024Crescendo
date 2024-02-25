package frc.robot.encoder;

import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;

public class RevAbsoluteEncoder implements MoEncoder.Encoder {
    public static Time VELOCITY_BASE_UNIT = Units.Seconds;

    private AbsoluteEncoder encoder;

    public RevAbsoluteEncoder(AbsoluteEncoder encoder) {
        this.encoder = encoder;
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
    }

    @Override
    public void setVelocityFactor(double factor) {
        encoder.setVelocityConversionFactor(factor);
    }

    @Override
    public Time getVelocityBaseUnit() {
        return VELOCITY_BASE_UNIT;
    }
}
