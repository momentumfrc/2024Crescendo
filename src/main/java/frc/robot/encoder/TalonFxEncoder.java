package frc.robot.encoder;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;

public class TalonFxEncoder implements MoEncoder.Encoder {
    public static final Time VELOCITY_BASE_UNIT = Units.Seconds;

    private final TalonFX talon;

    private double positionFactor = 1;
    private double velocityTimeFactor = 1;

    public TalonFxEncoder(TalonFX talon) {
        talon.getConfigurator()
                .apply(new FeedbackConfigs().withRotorToSensorRatio(1).withSensorToMechanismRatio(1));
        this.talon = talon;
    }

    @Override
    public double getPosition() {
        return talon.getPosition().getValueAsDouble();
    }

    @Override
    public void setPosition(double position) {
        talon.setPosition(position);
    }

    @Override
    public double getVelocity() {
        return talon.getVelocity().getValueAsDouble() * velocityTimeFactor;
    }

    @Override
    public void setPositionFactor(double factor) {
        talon.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(1 / factor));
        positionFactor = factor;
    }

    @Override
    public void setVelocityFactor(double factor) {
        // The talon doesn't support setting an separate velocity factor, so instead we save the difference between
        // the position and velocity factors, then add it on ourselves when returning the velocity.
        velocityTimeFactor = factor / positionFactor;
    }

    @Override
    public Time getVelocityBaseUnit() {
        return VELOCITY_BASE_UNIT;
    }
}
