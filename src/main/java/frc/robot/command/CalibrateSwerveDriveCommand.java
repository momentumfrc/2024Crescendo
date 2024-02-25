// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.util.MoPrefs;
import frc.robot.util.MoUnits;
import frc.robot.util.MoUtils;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;

public class CalibrateSwerveDriveCommand extends Command {
    private static class Calibrator {
        private static final double CALIBRATE_END = 0.8 * Math.PI;
        private static final double CALIBRATE_START = -0.8 * Math.PI;
        private static final int MIN_DATAPOINTS = 10;

        private static class DataPoint {
            public final double abs;
            public final double rel;

            public DataPoint(double abs, double rel) {
                this.abs = abs;
                this.rel = rel;
            }
        }

        private ArrayList<DataPoint> data = new ArrayList<>();

        private DoubleSupplier absEncoder;
        private DoubleSupplier relEncoder;

        double absZero;
        double relZero;

        public Calibrator(DoubleSupplier absEncoder, DoubleSupplier relEncoder) {
            this.absEncoder = absEncoder;
            this.relEncoder = relEncoder;
        }

        public void start() {
            data.clear();
            absZero = absEncoder.getAsDouble();
            relZero = relEncoder.getAsDouble();
        }

        /**
         * Gets the current position of the absolute encoder, offset by the initial zero, in radians
         * constrained between [-pi, pi)
         *
         * @returns The absolute position in radians
         */
        private double getAbsRad() {
            double rots = (absEncoder.getAsDouble() + 1 - absZero) % 1;
            return MoUtils.rotToRad(rots);
        }

        /**
         * Gets the current position of the relative encoder, offset by the initial zero, in
         * radians.
         *
         * @return The relative position in radians
         */
        private double getRelRad() {
            return relEncoder.getAsDouble() - relZero;
        }

        public boolean isFinished() {
            return getAbsRad() > CALIBRATE_END;
        }

        public void recordDataPoint() {
            DataPoint datum = new DataPoint(getAbsRad(), getRelRad());
            if (datum.abs > CALIBRATE_END || datum.abs < CALIBRATE_START) {
                return;
            }
            data.add(datum);
        }

        /**
         * Uses least-squares regression to calculate the factor the current relative encoder scale
         * should be adjusted by to minimize the error between the relative and absolute encoders.
         *
         * <p>Let R be the relative encoder count, and A be the absolute position in radians (as
         * given by the absolute encoder). Also let B be the current encoder scale (in units of
         * radians per encoder count) and let F be the error factor such that R*B*F = A. It follows
         * that F = A / R*B. Find the line of best fit where x = A and y = R*B. The slope of this
         * line, m, estimates R*B / A. Thus, F = 1/m, and so the correction factor is the reciprocal
         * of the slope of the line of best fit where the absolute position is the x-axis and the
         * current estimated position is the y-axis.
         *
         * @return The factor the current relative encoder scale should be adjusted by to minimize
         *     the error between the relative and absolute encoders.
         */
        public double calculateCorrectionFactor() {
            double n = data.size();
            if (n < MIN_DATAPOINTS) {
                DriverStation.reportWarning("Cannot complete calibration: insufficient data", false);
                return 1;
            }

            double sumx = 0;
            double sumy = 0;
            double sumx2 = 0;
            double sumxy = 0;

            for (DataPoint point : data) {
                sumx += point.abs;
                sumy += point.rel;
                sumx2 += point.abs * point.abs;
                sumxy += point.abs * point.rel;
            }

            double lsrl_slope = ((n * sumxy) - (sumx * sumy)) / ((n * sumx2) - (sumx * sumx));

            if (lsrl_slope < 0) {
                DriverStation.reportError("Negative correction factor!", false);
                return 1;
            }

            return 1 / lsrl_slope;
        }
    }

    private static final double CALIBRATE_SPEED = 0.05;

    private final DriveSubsystem drive;
    private Calibrator frontLeft;
    private Calibrator frontRight;
    private Calibrator rearLeft;
    private Calibrator rearRight;

    public CalibrateSwerveDriveCommand(DriveSubsystem drive) {
        addRequirements(drive);
        this.drive = drive;

        frontLeft = new Calibrator(
                () -> drive.frontLeft.absoluteEncoder.getPosition().in(Units.Rotations),
                () -> drive.frontLeft.relativeEncoder.getPosition().in(Units.Rotations));
        frontRight = new Calibrator(
                () -> drive.frontRight.absoluteEncoder.getPosition().in(Units.Rotations),
                () -> drive.frontRight.relativeEncoder.getPosition().in(Units.Rotations));
        rearLeft = new Calibrator(
                () -> drive.rearLeft.absoluteEncoder.getPosition().in(Units.Rotations),
                () -> drive.rearLeft.relativeEncoder.getPosition().in(Units.Rotations));
        rearRight = new Calibrator(
                () -> drive.rearRight.absoluteEncoder.getPosition().in(Units.Rotations),
                () -> drive.rearRight.relativeEncoder.getPosition().in(Units.Rotations));
    }

    @Override
    public void initialize() {
        frontLeft.start();
        rearLeft.start();
        frontRight.start();
        rearRight.start();

        drive.doResetEncoders = false;
    }

    @Override
    public void execute() {
        if (!frontLeft.isFinished()) {
            drive.frontLeft.directDrive(CALIBRATE_SPEED, 0);
            frontLeft.recordDataPoint();
        } else {
            drive.frontLeft.directDrive(0, 0);
        }

        if (!frontRight.isFinished()) {
            drive.frontRight.directDrive(CALIBRATE_SPEED, 0);
            frontRight.recordDataPoint();
        } else {
            drive.frontRight.directDrive(0, 0);
        }

        if (!rearLeft.isFinished()) {
            drive.rearLeft.directDrive(CALIBRATE_SPEED, 0);
            rearLeft.recordDataPoint();
        } else {
            drive.frontRight.directDrive(0, 0);
        }

        if (!rearRight.isFinished()) {
            drive.rearRight.directDrive(CALIBRATE_SPEED, 0);
            rearRight.recordDataPoint();
        } else {
            drive.frontRight.directDrive(0, 0);
        }
    }

    @Override
    public boolean isFinished() {
        return frontLeft.isFinished() && frontRight.isFinished() && rearLeft.isFinished() && rearRight.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        new Thread(() -> {
                    if (frontLeft.isFinished()) {
                        double factor = frontLeft.calculateCorrectionFactor();
                        System.out.format(
                                "frontLeft: factor=%.2f old=%.2f new=%.2f\n",
                                factor,
                                MoPrefs.flRotScale.get().in(MoUnits.EncoderTicksPerRotation),
                                MoPrefs.flRotScale.get().times(factor).in(MoUnits.EncoderTicksPerRotation));
                        MoPrefs.flRotScale.set(MoPrefs.flRotScale.get().times(factor));
                    }

                    if (frontRight.isFinished()) {
                        double factor = frontRight.calculateCorrectionFactor();
                        System.out.format(
                                "frontRight: factor=%.2f old=%.2f new=%.2f\n",
                                factor,
                                MoPrefs.frRotScale.get().in(MoUnits.EncoderTicksPerRotation),
                                MoPrefs.frRotScale.get().times(factor).in(MoUnits.EncoderTicksPerRotation));
                        MoPrefs.frRotScale.set(MoPrefs.frRotScale.get().times(factor));
                    }

                    if (rearLeft.isFinished()) {
                        double factor = rearLeft.calculateCorrectionFactor();
                        System.out.format(
                                "rearLeft: factor=%.2f old=%.2f new=%.2f\n",
                                factor,
                                MoPrefs.rlRotScale.get().in(MoUnits.EncoderTicksPerRotation),
                                MoPrefs.rlRotScale.get().times(factor).in(MoUnits.EncoderTicksPerRotation));
                        MoPrefs.rlRotScale.set(MoPrefs.rlRotScale.get().times(factor));
                    }

                    if (rearRight.isFinished()) {
                        double factor = rearRight.calculateCorrectionFactor();
                        System.out.format(
                                "rearRight: factor=%.2f old=%.2f new=%.2f\n",
                                factor,
                                MoPrefs.rrRotScale.get().in(MoUnits.EncoderTicksPerRotation),
                                MoPrefs.rrRotScale.get().times(factor).in(MoUnits.EncoderTicksPerRotation));
                        MoPrefs.rrRotScale.set(MoPrefs.rrRotScale.get().times(factor));
                    }
                })
                .start();

        drive.doResetEncoders = true;
    }
}
