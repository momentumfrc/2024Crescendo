package frc.robot.encoder;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;

public class TalonFxEncoder implements MoEncoder.Encoder {
    public static final Time VELOCITY_BASE_UNIT = Units.Seconds;

    private final TalonFX talon;

    private double positionFactor = 1;
    private double velocityFactor = 1;

    public TalonFxEncoder(TalonFX talon) {
        talon.getConfigurator()
                .apply(new FeedbackConfigs().withRotorToSensorRatio(1).withSensorToMechanismRatio(1));
        this.talon = talon;
    }

    @Override
    public double getPosition() {
        return talon.getRotorPosition().getValueAsDouble() * positionFactor;
    }

    @Override
    public void setPosition(double position) {
        talon.setPosition(position);
    }

    @Override
    public double getVelocity() {
        return talon.getRotorVelocity().getValueAsDouble() * velocityFactor;
    }

    @Override
    public void setPositionFactor(double factor) {
        this.positionFactor = factor;
    }

    @Override
    public void setVelocityFactor(double factor) {
        this.velocityFactor = factor;
    }

    @Override
    public Time getVelocityBaseUnit() {
        return VELOCITY_BASE_UNIT;
    }
}
