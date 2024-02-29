package frc.robot.util;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;

/**
 * Used to determine the necessary wrist angle to score in the speaker given the current distance to the speaker.
 */
public class TargetAngleFinder {
    private static TargetAngleFinder instance = null;

    public static TargetAngleFinder getInstance() {
        if (instance == null) {
            instance = new TargetAngleFinder();
        }
        return instance;
    }

    private record CalibrationPoint(double distanceInMeters, double angleInRotations) {}

    // TODO: This should be filled out with empirically measured data
    private CalibrationPoint[] data = {
        new CalibrationPoint(1, 0.25), // TEST DATA DO NOT USE
        new CalibrationPoint(3, 0.3) // TEST DATA DO NOT USE
    };

    private MutableMeasure<Angle> mutAngle = MutableMeasure.zero(Units.Rotations);

    public Measure<Angle> getWristAngle(Measure<Distance> distToTarget) {
        CalibrationPoint point1 = null;
        CalibrationPoint point2 = null;

        double distInMeters = distToTarget.in(Units.Meters);

        if (distInMeters < data[0].distanceInMeters) {
            point1 = data[0];
            point2 = data[1];
        } else if (distInMeters > data[data.length - 1].distanceInMeters) {
            point1 = data[data.length - 2];
            point2 = data[data.length - 1];
        } else {
            for (int i = 1; i < data.length; i++) {
                if (distInMeters <= data[i].distanceInMeters) {
                    point1 = data[i - 1];
                    point2 = data[i];
                    break;
                }
            }
        }

        double slope = (point2.angleInRotations - point1.angleInRotations)
                / (point2.distanceInMeters - point1.distanceInMeters);

        double dx = distInMeters - point1.distanceInMeters;
        double dy = dx * slope;
        double angle = point1.angleInRotations + dy;

        return mutAngle.mut_replace(angle, Units.Rotations);
    }

    private TargetAngleFinder() {
        if (data.length < 2) {
            throw new IllegalStateException("Insufficient calibration data to instantiate TargetFinder");
        }

        for (int i = 1; i < data.length; i++) {
            if (data[i - 1].distanceInMeters >= data[i].distanceInMeters) {
                throw new IllegalStateException(
                        "TargetFinder data must be sorted in order of increasing distance to target");
            }
        }
    }
}
