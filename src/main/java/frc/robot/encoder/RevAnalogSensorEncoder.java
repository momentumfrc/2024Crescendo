package frc.robot.encoder;

import com.revrobotics.SparkAnalogSensor;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;

public class RevAnalogSensorEncoder implements MoEncoder.Encoder {
    public static Time VELOCITY_BASE_UNIT = Units.Seconds;

    private SparkAnalogSensor sensor;

    public RevAnalogSensorEncoder(SparkAnalogSensor sensor) {
        this.sensor = sensor;

        sensor.setPositionConversionFactor(1);
        sensor.setVelocityConversionFactor(1);
    }

    @Override
    public double getPosition() {
        return sensor.getPosition();
    }

    @Override
    public void setPosition(double position) {
        throw new UnsupportedOperationException("Cannot set position on an absolute encoder");
    }

    @Override
    public double getVelocity() {
        return sensor.getVelocity();
    }

    @Override
    public void setPositionFactor(double factor) {
        sensor.setPositionConversionFactor(factor);
        sensor.setVelocityConversionFactor(factor);
    }

    @Override
    public Time getVelocityBaseUnit() {
        return VELOCITY_BASE_UNIT;
    }

    @Override
    public void setInverted(boolean inverted) {
        sensor.setInverted(inverted);
    }
}
