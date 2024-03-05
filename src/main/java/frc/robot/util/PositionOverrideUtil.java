package frc.robot.util;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class PositionOverrideUtil {

    private final Supplier<Measure<Angle>> currentPositionSupplier;
    private final Consumer<Measure<Angle>> setSmartMotion;
    private final Consumer<Measure<Velocity<Angle>>> setVelocity;
    MoPrefs.UnitPref<Velocity<Angle>> maxVelocityPref;
    private final double overrideTimeout;

    private boolean justFinishedAdjust = false;
    private boolean inPositionOverride = false;
    private Timer overridePosTimer = new Timer();

    private MutableMeasure<Angle> positionOverride = MutableMeasure.zero(Units.Rotations);
    private MutableMeasure<Velocity<Angle>> velocityOverride = MutableMeasure.zero(Units.RotationsPerSecond);

    public PositionOverrideUtil(
            Supplier<Measure<Angle>> currentPositionSupplier,
            Consumer<Measure<Angle>> setSmartMotion,
            Consumer<Measure<Velocity<Angle>>> setVelocity,
            MoPrefs.UnitPref<Velocity<Angle>> maxVelocityPref,
            double overrideTimeout) {
        this.currentPositionSupplier = currentPositionSupplier;
        this.setSmartMotion = setSmartMotion;
        this.setVelocity = setVelocity;
        this.maxVelocityPref = maxVelocityPref;
        this.overrideTimeout = overrideTimeout;
    }

    public void reset() {
        justFinishedAdjust = false;
        inPositionOverride = false;

        overridePosTimer.stop();
        overridePosTimer.reset();
    }

    public boolean getInPositionOverride() {
        return inPositionOverride;
    }

    public void runSmartMotionWithAdjust(Measure<Angle> position, double adjustPower) {
        if (adjustPower == 0) {
            if (justFinishedAdjust) {
                justFinishedAdjust = false;
                positionOverride.mut_replace(currentPositionSupplier.get());
            }
            if (inPositionOverride) {
                if (overridePosTimer.hasElapsed(overrideTimeout)) {
                    inPositionOverride = false;
                    runSmartMotionWithAdjust(position, adjustPower);
                    return;
                }

                setSmartMotion.accept(positionOverride);
            } else {
                setSmartMotion.accept(position);
            }
        } else {
            inPositionOverride = true;
            justFinishedAdjust = true;

            velocityOverride.mut_replace(maxVelocityPref.get());
            velocityOverride.mut_times(adjustPower);

            setVelocity.accept(velocityOverride);
        }
    }

    public void runVelocity(double adjustPower) {
        velocityOverride.mut_replace(maxVelocityPref.get());
        velocityOverride.mut_times(adjustPower);

        setVelocity.accept(velocityOverride);
    }
}
