// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.component.Limelight;
import frc.robot.util.MoShuffleboard;
import java.util.Map;

/** Subsystem that determines the robot's position on the field. */
public class PositioningSubsystem extends SubsystemBase {
    /**
     * The maximum acceptable distance, in meters, between a limelight position update and the
     * robot's current odometry.
     */
    private static final double POSITION_MAX_ACCEPTABLE_UPDATE_DELTA = 5;

    /** The limelight. Should be used by auto scoring commands for fine targeting. */
    public final Limelight limelight = new Limelight();

    private Pose2d robotPose = new Pose2d();

    private Field2d field = MoShuffleboard.getInstance().field;

    private GenericEntry didEstablishInitialPosition = MoShuffleboard.getInstance()
            .matchTab
            .add("Initial Position", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .getEntry();

    private GenericEntry shouldUseAprilTags = MoShuffleboard.getInstance()
            .settingsTab
            .add("Detect AprilTags", true)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();

    private final AHRS gyro;
    private final DriveSubsystem drive;

    private SwerveDriveOdometry odometry;

    private static enum FieldOrientedDriveMode {
        GYRO,
        ODOMETRY,
        NONE
    };

    private SendableChooser<FieldOrientedDriveMode> fieldOrientedDriveMode =
            MoShuffleboard.enumToChooser(FieldOrientedDriveMode.class);

    private Rotation2d fieldOrientedFwd;

    public PositioningSubsystem(AHRS ahrs, DriveSubsystem drive) {
        this.gyro = ahrs;
        this.drive = drive;

        MoShuffleboard.getInstance().settingsTab.add("Field Oriented Mode", fieldOrientedDriveMode);

        odometry = new SwerveDriveOdometry(drive.kinematics, gyro.getRotation2d(), drive.getWheelPositions());

        resetFieldOrientedFwd();

        var posGroup = MoShuffleboard.getInstance()
                .matchTab
                .getLayout("Relative Pos", BuiltInLayouts.kList)
                .withSize(2, 1)
                .withProperties(Map.of("Label position", "RIGHT"));
        posGroup.addDouble("X", () -> robotPose.getX());
        posGroup.addDouble("Y", () -> robotPose.getY());
        posGroup.addDouble("Rot", () -> robotPose.getRotation().getDegrees());
    }

    public Rotation2d getFieldOrientedDriveHeading() {
        var foMode = fieldOrientedDriveMode.getSelected();
        switch (foMode) {
            case GYRO:
                return gyro.getRotation2d().minus(fieldOrientedFwd);
            case ODOMETRY:
                return getRobotPose().getRotation();
            case NONE:
            default:
                return new Rotation2d();
        }
    }

    /**
     * Get robot pose, relative to the origin at the blue alliance.
     */
    public Pose2d getRobotPose() {
        return robotPose;
    }

    public boolean hasInitialPosition() {
        return this.didEstablishInitialPosition.getBoolean(false);
    }

    public void setRobotPose(Pose2d pose) {
        if (this.didEstablishInitialPosition.getBoolean(false)
                && this.odometry.getPoseMeters().getTranslation().getDistance(pose.getTranslation())
                        > POSITION_MAX_ACCEPTABLE_UPDATE_DELTA) {
            return;
        }
        this.didEstablishInitialPosition.setBoolean(true);
        this.odometry.resetPosition(gyro.getRotation2d(), drive.getWheelPositions(), pose);
    }

    public void resetFieldOrientedFwd() {
        this.fieldOrientedFwd = gyro.getRotation2d();
    }

    @Override
    public void periodic() {
        limelight.periodic();

        limelight.getRobotPose().ifPresent(pose -> {
            if (!shouldUseAprilTags.getBoolean(true)) {
                return;
            }
            if (drive.isMoving()) {
                return;
            }
            this.setRobotPose(pose.toPose2d());
        });

        robotPose = odometry.update(gyro.getRotation2d(), drive.getWheelPositions());
        field.setRobotPose(getRobotPose());

        if (fieldOrientedDriveMode.getSelected() != FieldOrientedDriveMode.GYRO) resetFieldOrientedFwd();
    }
}
