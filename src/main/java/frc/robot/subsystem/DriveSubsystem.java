// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.momentum4999.motune.PIDTuner;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.component.SwerveModule;
import frc.robot.util.MoPIDF;
import frc.robot.util.MoPrefs;
import frc.robot.util.MoShuffleboard;
import frc.robot.util.TunerUtils;

public class DriveSubsystem extends SubsystemBase {
    /**
     * The maximum rate of turn that the drive will consider as equivalent to zero. Used to
     * determine when to re-enable heading pid after executing a driver-requested turn.
     */
    private static final double TURN_RATE_CUTOFF = 0.001;

    /** The maximum rate of movement that the drive will consider as equivalent to zero. */
    private static final double MOVE_RATE_CUTOFF = 0.05;

    private static final double RESET_ENCODER_INTERVAL = 0.5;

    /**
     * The distance along one axis, in meters, from the center of the robot to the center of a
     * swerve module wheel. Since the chassis is square, we don't have to keep track of the X and Y
     * dimensions separately.
     * <p>
     * The robot frame has a side length of 29.5". The mounting holes for the serve modules
     * are 0.5" inset from the edge of the frame. Per the SDS layout diagram of the MK4 swerve
     * modules, the length between the mounting holes and the center of the swerve wheel is 2.75".
     * So, the distance from the center of the robot to the center of a swerve wheel is
     * 29.5"/2 - 0.5" - 2.75" = 11.5".
     */
    private static final double SWERVE_WHEEL_OFFSET = 11.5 * 0.0254;

    private static enum TurnState {
        TURNING,
        HOLD_HEADING
    };

    private TurnState turnState = TurnState.HOLD_HEADING;
    private Rotation2d maintainHeading;

    private final MoPIDF headingController = new MoPIDF();
    private final PIDTuner headingTuner = TunerUtils.forMoPIDF(headingController, "Drive Heading");

    private GenericSubscriber shouldHeadingPID = MoShuffleboard.getInstance()
            .settingsTab
            .add("Keep Heading", false)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();

    public final SwerveModule frontLeft;
    public final SwerveModule frontRight;
    public final SwerveModule rearLeft;
    public final SwerveModule rearRight;

    private final Timer resetEncoderTimer = new Timer();

    public final MoPIDF xPathController = new MoPIDF();
    public final MoPIDF yPathController = new MoPIDF();
    public final MoPIDF rotPathController = new MoPIDF();

    private final PIDTuner xPathTuner = TunerUtils.forMoPID(xPathController, "Follow Path X");
    private final PIDTuner yPathTuner = TunerUtils.forMoPID(xPathController, "Follow Path Y");
    private final PIDTuner rotPathTuner = TunerUtils.forMoPID(xPathController, "Follow Path Rot");

    public final SwerveDriveKinematics kinematics;

    private final AHRS gyro;

    public boolean doResetEncoders = true;

    public DriveSubsystem(AHRS gyro) {
        this.gyro = gyro;
        maintainHeading = getCurrHeading();

        headingController.enableContinuousInput(-Math.PI, Math.PI);

        this.frontLeft = new SwerveModule(
                "FL",
                new CANSparkMax(Constants.TURN_LEFT_FRONT.address, MotorType.kBrushless),
                new TalonFX(Constants.DRIVE_LEFT_FRONT.address),
                MoPrefs.flZero,
                MoPrefs.flScale,
                MoPrefs.flDriveMtrScale);

        this.frontRight = new SwerveModule(
                "FR",
                new CANSparkMax(Constants.TURN_RIGHT_FRONT.address, MotorType.kBrushless),
                new TalonFX(Constants.DRIVE_RIGHT_FRONT.address),
                MoPrefs.frZero,
                MoPrefs.frScale,
                MoPrefs.frDriveMtrScale);

        this.rearLeft = new SwerveModule(
                "RL",
                new CANSparkMax(Constants.TURN_LEFT_REAR.address, MotorType.kBrushless),
                new TalonFX(Constants.DRIVE_LEFT_REAR.address),
                MoPrefs.rlZero,
                MoPrefs.rlScale,
                MoPrefs.rlDriveMtrScale);

        this.rearRight = new SwerveModule(
                "RR",
                new CANSparkMax(Constants.TURN_RIGHT_REAR.address, MotorType.kBrushless),
                new TalonFX(Constants.DRIVE_RIGHT_REAR.address),
                MoPrefs.rrZero,
                MoPrefs.rrScale,
                MoPrefs.rrDriveMtrScale);

        resetEncoderTimer.start();

        MoShuffleboard.getInstance()
                .matchTab
                .addDouble(
                        "FL_POS", () -> frontLeft.driveMotor.getRotorPosition().getValueAsDouble());
        MoShuffleboard.getInstance()
                .matchTab
                .addDouble(
                        "FL_POS_m",
                        () -> frontLeft.driveMotor.getRotorPosition().getValueAsDouble()
                                / MoPrefs.flDriveMtrScale.get());

        Translation2d fl = new Translation2d(SWERVE_WHEEL_OFFSET, SWERVE_WHEEL_OFFSET);
        Translation2d fr = new Translation2d(SWERVE_WHEEL_OFFSET, -SWERVE_WHEEL_OFFSET);
        Translation2d rl = new Translation2d(-SWERVE_WHEEL_OFFSET, SWERVE_WHEEL_OFFSET);
        Translation2d rr = new Translation2d(-SWERVE_WHEEL_OFFSET, -SWERVE_WHEEL_OFFSET);

        this.kinematics = new SwerveDriveKinematics(fl, fr, rl, rr);
    }

    public SwerveModulePosition[] getWheelPositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(), frontRight.getPosition(), rearLeft.getPosition(), rearRight.getPosition()
        };
    }

    /**
     * Gets the current heading, within the range (-PI, PI]
     *
     * @return the current heading
     */
    private Rotation2d getCurrHeading() {
        return Rotation2d.fromRadians(MathUtil.angleModulus(gyro.getRotation2d().getRadians()));
    }

    /**
     * Calculate how much the robot should turn. We want to use PID to prevent any turning, except
     * for these situations: (1) if the driver has requested a turn, or (2) if the robot is slowing
     * down after a requested turn (to prevent an unexpected 'snap-back' behavior).
     *
     * @param turnRequest The turn requested by the driver
     * @param currentHeading The robot's current heading, as reported by the gyro
     * @return How much the robot should turn
     */
    private double calculateTurn(double turnRequest, Rotation2d currentHeading) {
        switch (turnState) {
            case HOLD_HEADING:
                if (turnRequest != 0) {
                    turnState = TurnState.TURNING;
                }
                break;
            case TURNING:
                if (turnRequest == 0 && Math.abs(gyro.getRate()) < TURN_RATE_CUTOFF) {
                    maintainHeading = currentHeading;
                    turnState = TurnState.HOLD_HEADING;
                }
                break;
        }
        switch (turnState) {
            case HOLD_HEADING:
                if (shouldHeadingPID.getBoolean(true)) {
                    return headingController.calculate(currentHeading.getRadians(), maintainHeading.getRadians());
                }
            case TURNING:
            default:
                return turnRequest;
        }
    }

    public void driveCartesian(double fwdRequest, double leftRequest, double turnRequest) {
        this.driveCartesian(fwdRequest, leftRequest, turnRequest, new Rotation2d());
    }

    public void driveCartesian(
            double fwdRequest, double leftRequest, double turnRequest, Rotation2d fieldOrientedDriveAngle) {
        turnRequest = calculateTurn(turnRequest, getCurrHeading());

        double maxLinearSpeed = MoPrefs.maxDriveSpeed.get();
        double maxAngularSpeed = MoPrefs.maxTurnSpeed.get();

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                -fwdRequest * maxLinearSpeed,
                leftRequest * maxLinearSpeed,
                turnRequest * maxAngularSpeed,
                fieldOrientedDriveAngle);

        driveSwerveStates(kinematics.toSwerveModuleStates(speeds));
    }

    public void stop() {
        frontLeft.driveMotor.stopMotor();
        frontRight.driveMotor.stopMotor();
        rearLeft.driveMotor.stopMotor();
        rearRight.driveMotor.stopMotor();
    }

    public void driveSwerveStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MoPrefs.maxDriveSpeed.get());

        frontLeft.drive(states[0]);
        frontRight.drive(states[1]);
        rearLeft.drive(states[2]);
        rearRight.drive(states[3]);
    }

    public boolean isMoving() {
        return (frontLeft.driveMotor.getRotorVelocity().getValueAsDouble() / MoPrefs.flDriveMtrScale.get())
                        > MOVE_RATE_CUTOFF
                || (frontRight.driveMotor.getRotorVelocity().getValueAsDouble() / MoPrefs.frDriveMtrScale.get())
                        > MOVE_RATE_CUTOFF
                || (rearLeft.driveMotor.getRotorVelocity().getValueAsDouble() / MoPrefs.rlDriveMtrScale.get())
                        > MOVE_RATE_CUTOFF
                || (rearRight.driveMotor.getRotorVelocity().getValueAsDouble() / MoPrefs.rrDriveMtrScale.get())
                        > MOVE_RATE_CUTOFF;
    }

    public void resetRelativeEncoders() {
        if (!doResetEncoders) return;
        frontLeft.setRelativePosition();
        frontRight.setRelativePosition();
        rearLeft.setRelativePosition();
        rearRight.setRelativePosition();
    }

    @Override
    public void periodic() {
        if (resetEncoderTimer.advanceIfElapsed(RESET_ENCODER_INTERVAL)) {
            resetRelativeEncoders();
        }

        if (DriverStation.isDisabled()) {
            this.stop();
        }
    }
}
