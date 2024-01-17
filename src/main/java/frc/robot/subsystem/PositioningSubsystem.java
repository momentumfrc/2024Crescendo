// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    /**
     * The size of the field, in meters. Used to transform alliance-relative coordinates into
     * field-relative coordinates.
     */
    private static final Translation2d fieldSize = new Translation2d(16.54175, 8.0137);

    /** The limelight. Should be used by auto scoring commands for fine targeting. */
    public final Limelight limelight = new Limelight();

    private Pose2d robotPose = new Pose2d();

    private Field2d field = MoShuffleboard.getInstance().field;

    private GenericEntry didEstablishInitialPosition =
            MoShuffleboard.getInstance()
                    .matchTab
                    .add("Initial Position", false)
                    .withWidget(BuiltInWidgets.kBooleanBox)
                    .getEntry();

    private GenericEntry shouldUseAprilTags =
            MoShuffleboard.getInstance()
                    .settingsTab
                    .add("Detect AprilTags", true)
                    .withWidget(BuiltInWidgets.kToggleSwitch)
                    .getEntry();

    private Transform2d limelightTransform =
            new Transform2d(new Translation2d(), Rotation2d.fromRotations(0.5));

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

        odometry =
                new SwerveDriveOdometry(
                        drive.kinematics, gyro.getRotation2d(), drive.getWheelPositions());

        resetFieldOrientedFwd();

        var posGroup =
                MoShuffleboard.getInstance()
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
     * Get robot pose in alliance coordinates.
     *
     * <p>Note that the robot always assumes its origin is in the right corner of its alliance.
     */
    public Pose2d getRobotPose() {
        return robotPose;
    }

    /** Get the robot pose in field coordinates. */
    public Pose2d getAbsoluteRobotPose() {
        if (DriverStation.getAlliance().orElse(null) == Alliance.Blue) {
            return robotPose;
        }
        Pose2d alliancePose = robotPose;
        Translation2d translation = fieldSize.minus(alliancePose.getTranslation());
        Rotation2d rotation = robotPose.getRotation().rotateBy(Rotation2d.fromRotations(0.5));
        return new Pose2d(translation, rotation);
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

        limelight
                .getRobotPose()
                .ifPresent(
                        pose -> {
                            if (!shouldUseAprilTags.getBoolean(true)) {
                                return;
                            }
                            if (drive.isMoving()) {
                                return;
                            }
                            var transformedPose = pose.toPose2d().transformBy(limelightTransform);
                            this.setRobotPose(transformedPose);
                        });

        robotPose = odometry.update(gyro.getRotation2d(), drive.getWheelPositions());
        field.setRobotPose(getAbsoluteRobotPose());

        if (fieldOrientedDriveMode.getSelected() != FieldOrientedDriveMode.GYRO)
            resetFieldOrientedFwd();
    }
}
