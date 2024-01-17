// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.component;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.PoseFilter;
import java.util.Optional;

public class Limelight {
    /**
     * The number of poses reported by the limelight which will be considered in order to filter out
     * invalid data.
     */
    private static final int LIMELIGHT_DATAPOINTS = 10;

    /**
     * The maximum standard deviation of saved poses reported by the limelight allowed before the
     * dataset is considered invalid.
     */
    private static final double STDDEV_CUTOFF = 0.01;

    /** The maximum z-score of any saved pose before it is considered invalid. */
    private static final double ZSCORE_CUTOFF = 3;

    private static final double DEGS_TO_RADS = Math.PI / 180;

    private PoseFilter poseFilter =
            new PoseFilter(LIMELIGHT_DATAPOINTS, STDDEV_CUTOFF, ZSCORE_CUTOFF);
    private Optional<Pose3d> lastReportedPose = Optional.empty();
    private Optional<Translation2d> lastReportedCrosshair = Optional.empty();

    private final NetworkTable limelightTable =
            NetworkTableInstance.getDefault().getTable("limelight");
    private final DoublePublisher pipelinePublisher =
            limelightTable.getDoubleTopic("pipeline").publish();
    private final DoublePublisher ledPublisher = limelightTable.getDoubleTopic("ledMode").publish();
    private final DoubleSubscriber tvSubscriber = limelightTable.getDoubleTopic("tv").subscribe(0);
    private final DoubleSubscriber txSubscriber = limelightTable.getDoubleTopic("tx").subscribe(0);
    private final DoubleSubscriber tySubscriber = limelightTable.getDoubleTopic("ty").subscribe(0);

    private final DoubleArraySubscriber botposeBlueSubscriber =
            limelightTable.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[6]);
    private final DoubleArraySubscriber botposeRedSubscriber =
            limelightTable.getDoubleArrayTopic("botpose_wpired").subscribe(new double[6]);

    public Optional<Pose3d> getRobotPose() {
        return lastReportedPose;
    }

    public Optional<Translation2d> getCrosshair() {
        return lastReportedCrosshair;
    }

    /**
     * Pose data is considered valid if it satisfies the following conditions:
     *
     * <ol>
     *   <li>It has a length of exactly 6
     *   <li>At least one value is nonzero
     * </ol>
     */
    private boolean isPoseDataValid(double[] rawPoseData) {
        if (rawPoseData.length != 6) return false;
        for (double data : rawPoseData) {
            if (data != 0) return true;
        }
        return false;
    }

    public void periodic() {
        // Publish data to Limelight
        pipelinePublisher.accept(0);
        ledPublisher.accept(0);

        // Receive data from Limelight
        boolean hasDetection = tvSubscriber.get() > 0;
        Translation2d crosshairs = new Translation2d(txSubscriber.get(), tySubscriber.get());
        double[] rawPoseData;
        if (DriverStation.getAlliance().orElse(null) == Alliance.Red) {
            rawPoseData = botposeRedSubscriber.get();
        } else {
            rawPoseData = botposeBlueSubscriber.get();
        }

        // Process received data
        if (!hasDetection) {
            this.lastReportedCrosshair = Optional.empty();
            this.lastReportedPose = Optional.empty();
            this.poseFilter.clear();
            return;
        } else {
            this.lastReportedCrosshair = Optional.of(crosshairs);

            if (isPoseDataValid(rawPoseData)) {
                Pose3d pose =
                        new Pose3d(
                                new Translation3d(rawPoseData[0], rawPoseData[1], rawPoseData[2]),
                                new Rotation3d(
                                        rawPoseData[3] * DEGS_TO_RADS,
                                        rawPoseData[4] * DEGS_TO_RADS,
                                        rawPoseData[5] * DEGS_TO_RADS));
                this.lastReportedPose = poseFilter.accept(pose);
            } else {
                this.lastReportedPose = Optional.empty();
                this.poseFilter.clear();
            }
        }
    }
}
