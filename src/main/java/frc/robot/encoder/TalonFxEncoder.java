package frc.robot.encoder;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;

public class TalonFxEncoder implements MoEncoder.Encoder {
    public static final Time VELOCITY_BASE_UNIT = Units.Seconds;

    private final TalonFX talon;

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
        return talon.getVelocity().getValueAsDouble();
    }

    @Override
    public void setPositionFactor(double factor) {
        talon.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(1 / factor));
    }

    @Override
    public Time getVelocityBaseUnit() {
        return VELOCITY_BASE_UNIT;
    }
}
