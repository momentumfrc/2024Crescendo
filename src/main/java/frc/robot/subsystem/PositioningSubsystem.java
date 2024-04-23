// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.component.LimelightHelpers;
import frc.robot.util.MoShuffleboard;
import java.util.EnumMap;
import java.util.Map;

/** Subsystem that determines the robot's position on the field. */
public class PositioningSubsystem extends SubsystemBase {
    /**
     * The maximum acceptable distance, in meters, between a limelight position update and the
     * robot's current odometry.
     */
    private static final double POSITION_MAX_ACCEPTABLE_UPDATE_DELTA = 5;

    private Pose2d robotPose = new Pose2d();

    private Field2d field = MoShuffleboard.getInstance().field;

    private EnumMap<DriverStation.Alliance, Pose2d> speakerPoses = new EnumMap<>(DriverStation.Alliance.class);

    private GenericEntry didEstablishInitialPosition = MoShuffleboard.getInstance()
            .driveTab
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

    private SwerveDrivePoseEstimator estimator;

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

        estimator = new SwerveDrivePoseEstimator(
                drive.kinematics, gyro.getRotation2d(), drive.getWheelPositions(), new Pose2d());

        resetFieldOrientedFwd();

        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
        speakerPoses.put(DriverStation.Alliance.Blue, layout.getTagPose(7).get().toPose2d());
        speakerPoses.put(DriverStation.Alliance.Red, layout.getTagPose(4).get().toPose2d());

        var posGroup = MoShuffleboard.getInstance()
                .driveTab
                .getLayout("Relative Pos", BuiltInLayouts.kList)
                .withSize(2, 1)
                .withProperties(Map.of("Label position", "RIGHT"));
        posGroup.addDouble("X", () -> robotPose.getX());
        posGroup.addDouble("Y", () -> robotPose.getY());
        posGroup.addDouble("Rot", () -> robotPose.getRotation().getDegrees());
    }

    public Pose2d getSpeakerPose() {
        return speakerPoses.get(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue));
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
                && this.estimator.getEstimatedPosition().getTranslation().getDistance(pose.getTranslation())
                        > POSITION_MAX_ACCEPTABLE_UPDATE_DELTA) {
            return;
        }
        this.didEstablishInitialPosition.setBoolean(true);
        this.estimator.resetPosition(gyro.getRotation2d(), drive.getWheelPositions(), pose);
    }

    public void resetFieldOrientedFwd() {
        this.fieldOrientedFwd = gyro.getRotation2d().plus(Rotation2d.fromRotations(0.5));
    }

    @Override
    public void periodic() {
        // See:
        // https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2#using-wpilibs-pose-estimator
        LimelightHelpers.SetRobotOrientation(
                "limelight", estimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate llPos = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (Math.abs(gyro.getRate()) < 720 && llPos.tagCount > 0) {
            if (!hasInitialPosition()) {
                setRobotPose(llPos.pose);
            } else {
                estimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
                estimator.addVisionMeasurement(llPos.pose, llPos.timestampSeconds);
            }
        }

        robotPose = estimator.update(gyro.getRotation2d(), drive.getWheelPositions());
        field.setRobotPose(robotPose);

        if (fieldOrientedDriveMode.getSelected() != FieldOrientedDriveMode.GYRO) resetFieldOrientedFwd();
    }
}
