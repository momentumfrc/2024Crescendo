package frc.robot.util;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Per;
import edu.wpi.first.units.Units;

public class MoUnits {
    private MoUnits() {
        throw new UnsupportedOperationException("Cannot instantiate MoUnits");
    }

    public static Dimensionless EncoderTicks = Units.derive(Units.Value)
            .fromBase((v) -> v)
            .toBase((v) -> v)
            .named("EncoderTicks")
            .symbol("T")
            .make();

    public static Per<Dimensionless, Distance> EncoderTicksPerMeter = EncoderTicks.per(Units.Meters);
    public static Per<Dimensionless, Angle> EncoderTicksPerRotation = EncoderTicks.per(Units.Rotations);
    public static Per<Dimensionless, Angle> EncoderTicksPerRadian = EncoderTicks.per(Units.Radians);
}
