package frc.robot.encoder;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;

public class RevRelativeEncoder implements MoEncoder.Encoder {
    public static final Time VELOCITY_BASE_UNIT = Units.Minute;

    private RelativeEncoder encoder;

    public RevRelativeEncoder(RelativeEncoder encoder) {
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
        encoder.setPosition(position);
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
