package frc.robot.util;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Per;
import edu.wpi.first.units.UnaryFunction;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;

public class MoUnits {
    private MoUnits() {
        throw new UnsupportedOperationException("Cannot instantiate MoUnits");
    }

    public static class EncoderAngle extends Unit<EncoderAngle> {
        EncoderAngle(double baseUnitEquivalent, String name, String symbol) {
            super(EncoderAngle.class, baseUnitEquivalent, name, symbol);
        }

        EncoderAngle(UnaryFunction toBaseConverter, UnaryFunction fromBaseConverter, String name, String symbol) {
            super(EncoderAngle.class, toBaseConverter, fromBaseConverter, name, symbol);
        }
    }

    public static EncoderAngle EncoderTicks = new EncoderAngle(1, "Encoder_Ticks", "Et");

    public static Per<EncoderAngle, Distance> EncoderTicksPerMeter = EncoderTicks.per(Units.Meters);
    public static Per<EncoderAngle, Distance> EncoderTicksPerCentimeter = EncoderTicks.per(Units.Centimeters);
    public static Per<EncoderAngle, Angle> EncoderTicksPerRotation = EncoderTicks.per(Units.Rotations);
    public static Per<EncoderAngle, Angle> EncoderTicksPerRadian = EncoderTicks.per(Units.Radians);
}
