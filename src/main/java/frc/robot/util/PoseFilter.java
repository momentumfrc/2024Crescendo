// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.Optional;

/** Filters a stream of Pose3d to remove outliers and potentially bad data. */
public class PoseFilter {
    private int size;
    private double stddevCutoff;
    private double zscoreCutoff;

    private final Deque<Pose3d> poseData;
    private Translation3d centroid = new Translation3d();
    private double[] centroidDistances;
    private List<Pose3d> goodData;

    /**
     * Constructs a PoseFilter.
     *
     * @param size How many poses to keep track of and consider
     * @param stddevCutoff The maximum standard deviation allowed before the whole dataset is
     *     considered invalid
     * @param zscoreCutoff The maximum z-score of any individual data point before it is considered
     *     invalid and ignored
     */
    public PoseFilter(int size, double stddevCutoff, double zscoreCutoff) {
        this.size = size;
        this.stddevCutoff = stddevCutoff;
        this.zscoreCutoff = zscoreCutoff;

        this.poseData = new ArrayDeque<>(size + 1);
        this.centroidDistances = new double[size];
        this.goodData = new ArrayList<>(size);
    }

    /**
     * Process the next pose.
     *
     * @param nextPose The next pose to process
     * @return The last valid pose, or Optional.empty() if a valid pose cannot be determined.
     */
    public Optional<Pose3d> accept(Pose3d nextPose) {
        // We use a rolling average to calculate the centroid
        centroid = centroid.plus(nextPose.getTranslation().div(size));

        poseData.addLast(nextPose);
        if (poseData.size() < this.size + 1) {
            return Optional.empty();
        }
        Pose3d removeData = poseData.removeFirst();

        // When we remove a datapoint, we need to remove it from the rolling average
        centroid = centroid.minus(removeData.getTranslation().div(size));

        // Calculate the mean and standard deviation of distances to the centroid
        int i = 0;
        for (Pose3d pose : poseData) {
            centroidDistances[i++] = pose.getTranslation().getDistance(centroid);
        }

        double mean = 0;
        for (double dist : centroidDistances) {
            mean += dist;
        }
        mean /= this.size;

        double stddev = 0;
        for (double dist : centroidDistances) {
            stddev += Math.pow(dist - mean, 2);
        }
        stddev = Math.sqrt(stddev / (this.size - 1));

        // If the standard deviation is too high, there isn't enough good data to determine
        // an accurate position
        if (stddev > this.stddevCutoff) {
            return Optional.empty();
        }

        i = 0;
        goodData.clear();
        for (Pose3d pose : poseData) {
            double dist = centroidDistances[i++];
            double zscore = (dist - mean) / stddev;
            if (zscore < this.zscoreCutoff) {
                goodData.add(pose);
            }
        }

        if (goodData.size() == 0) {
            return Optional.empty();
        }

        Translation3d averageTranslation = new Translation3d();
        double x = 0;
        double y = 0;
        double z = 0;
        for (Pose3d pose : goodData) {
            averageTranslation = averageTranslation.plus(pose.getTranslation());
            x += pose.getRotation().getX();
            y += pose.getRotation().getY();
            z += pose.getRotation().getZ();
        }
        averageTranslation = averageTranslation.div(goodData.size());
        Rotation3d averageRotation = new Rotation3d(x / goodData.size(), y / goodData.size(), z / goodData.size());

        return Optional.of(new Pose3d(averageTranslation, averageRotation));
    }

    public void clear() {
        poseData.clear();
        centroid = new Translation3d();
    }
}
