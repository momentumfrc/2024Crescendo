package frc.robot.subsystem;

import com.momentum4999.motune.PIDTuner;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.encoder.MoEncoder;
import frc.robot.util.MoPrefs;
import frc.robot.util.MoShuffleboard;
import frc.robot.util.MoSparkMaxArmPID;
import frc.robot.util.MoSparkMaxPID;
import frc.robot.util.MoUtils;
import frc.robot.util.TunerUtils;
import java.util.Map;

public class ArmSubsystem extends SubsystemBase {
    private static final Measure<Current> SHOULDER_CURRENT_LIMIT = Units.Amps.of(50);
    private static final Measure<Current> WRIST_CURRENT_LIMIT = Units.Amps.of(50);

    private static final Measure<Time> WRIST_ENCODER_VALID_PERIOD = Units.Seconds.of(5);
    private static final double WRIST_ENCODER_INVALID_RANGE = 0.01;

    public static enum ArmControlMode {
        SMARTMOTION,
        DIRECT_VELOCITY,
        FALLBACK_DIRECT_POWER
    };

    private final CANSparkMax shoulderLeftMtr;
    private final CANSparkMax shoulderRightMtr;
    private final CANSparkMax wristMtr;

    private final MoEncoder<Angle> shoulderAbsEncoder;
    private final MoEncoder<Angle> wristAbsEncoder;

    public final MoEncoder<Angle> shoulderRelEncoder;
    public final MoEncoder<Angle> wristRelEncoder;

    private final MoSparkMaxArmPID shoulderVelocityPid;
    private final MoSparkMaxArmPID wristVelocityPid;

    private final MoSparkMaxArmPID shoulderSmartMotionPid;
    private final MoSparkMaxArmPID wristSmartMotionPid;

    private final PIDTuner shoulderVelTuner;
    private final PIDTuner wristVelTuner;
    private final PIDTuner shoulderPosTuner;
    private final PIDTuner wristPosTuner;

    public final SendableChooser<ArmControlMode> controlMode;

    private final GenericEntry coastArmsEntry;
    boolean lastCoastArms = false;

    boolean trustWristEncoder = false;
    Timer trustWristEncoderTimer = new Timer();

    public static record ArmPosition(Measure<Angle> shoulderAngle, Measure<Angle> wristAngle) {}

    public static record ArmMovementRequest(double shoulderPower, double wristPower) {
        public ArmMovementRequest(double shoulderPower, double wristPower) {
            this.shoulderPower = MoUtils.clamp(shoulderPower, -1, 1);
            this.wristPower = MoUtils.clamp(wristPower, -1, 1);
        }

        public boolean isZero() {
            return Math.abs(shoulderPower) < Constants.FLOAT_EPSILON && Math.abs(wristPower) < Constants.FLOAT_EPSILON;
        }
    }

    public void configureMotors() {
        trustWristEncoder = false;

        System.out.println("CONFIGURE ARM MOTORS");
        shoulderLeftMtr.restoreFactoryDefaults();
        shoulderRightMtr.restoreFactoryDefaults();
        wristMtr.restoreFactoryDefaults();

        shoulderLeftMtr.setSmartCurrentLimit((int) SHOULDER_CURRENT_LIMIT.in(Units.Amps));
        shoulderRightMtr.setSmartCurrentLimit((int) SHOULDER_CURRENT_LIMIT.in(Units.Amps));
        wristMtr.setSmartCurrentLimit((int) WRIST_CURRENT_LIMIT.in(Units.Amps));

        shoulderLeftMtr.setIdleMode(IdleMode.kBrake);
        shoulderRightMtr.setIdleMode(IdleMode.kBrake);
        wristMtr.setIdleMode(IdleMode.kBrake);

        wristMtr.setInverted(true);

        shoulderLeftMtr.setInverted(true);
        shoulderRightMtr.follow(shoulderLeftMtr, true);

        shoulderAbsEncoder.setInverted(false);
        wristAbsEncoder.setInverted(true);

        MoUtils.setupRelativeEncoder(
                shoulderRelEncoder,
                shoulderAbsEncoder.getPosition(),
                MoPrefs.shoulderAbsZero.get(),
                MoPrefs.shoulderEncoderScale.get());

        MoUtils.setupRelativeEncoder(
                wristRelEncoder,
                wristAbsEncoder.getPosition(),
                MoPrefs.wristAbsZero.get(),
                MoPrefs.wristEncoderScale.get());

        shoulderAbsEncoder.setConversionFactor(MoPrefs.shoulderAbsEncoderScale.get());

        shoulderLeftMtr.setSoftLimit(SoftLimitDirection.kReverse, 0);
        wristMtr.setSoftLimit(SoftLimitDirection.kReverse, 0);

        shoulderLeftMtr.setSoftLimit(SoftLimitDirection.kForward, (float)
                MoPrefs.shoulderMaxExtension.get().in(shoulderRelEncoder.getInternalEncoderUnits()));
        wristMtr.setSoftLimit(SoftLimitDirection.kForward, (float)
                MoPrefs.wristMaxExtension.get().in(wristRelEncoder.getInternalEncoderUnits()));

        shoulderLeftMtr.enableSoftLimit(SoftLimitDirection.kReverse, true);
        shoulderLeftMtr.enableSoftLimit(SoftLimitDirection.kForward, true);
        wristMtr.enableSoftLimit(SoftLimitDirection.kReverse, true);
        wristMtr.enableSoftLimit(SoftLimitDirection.kForward, true);

        shoulderVelTuner.populatePIDValues();
        wristVelTuner.populatePIDValues();
        shoulderPosTuner.populatePIDValues();
        wristPosTuner.populatePIDValues();

        lastCoastArms = coastArmsEntry.getBoolean(false);
        IdleMode idleMode = lastCoastArms ? IdleMode.kCoast : IdleMode.kBrake;
        shoulderLeftMtr.setIdleMode(idleMode);
        shoulderRightMtr.setIdleMode(idleMode);
        wristMtr.setIdleMode(idleMode);

        trustWristEncoderTimer.start();
        MoShuffleboard.getInstance().armTab.addBoolean("Trust wrist encoder?", () -> trustWristEncoder);
    }

    public ArmSubsystem(ShooterSubsystem shooter) {
        super("Arm");
        shoulderLeftMtr = new CANSparkMax(Constants.SHOULDER_LEFT_MTR.address(), MotorType.kBrushless);
        shoulderRightMtr = new CANSparkMax(Constants.SHOULDER_RIGHT_MTR.address(), MotorType.kBrushless);
        wristMtr = new CANSparkMax(Constants.WRIST_MTR.address(), MotorType.kBrushless);

        // TODO: Ensure the shoulder abs encoder is wired to the left spark
        shoulderAbsEncoder = MoEncoder.forSparkAbsolute(
                shoulderLeftMtr.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle), Units.Rotations);

        // The absolute encoder for the wrist is plugged into the indexer motor controller, borrow it
        var rawWristBorrowedAbsEncoder = shooter.roller.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        wristAbsEncoder = MoEncoder.forSparkAbsolute(rawWristBorrowedAbsEncoder, Units.Rotations);

        shoulderRelEncoder = MoEncoder.forSparkRelative(shoulderLeftMtr.getEncoder(), Units.Rotations);
        wristRelEncoder = MoEncoder.forSparkRelative(wristMtr.getEncoder(), Units.Rotations);

        // Setup listeners for encoder scales and absolute zeros. Use notifyImmediately on zero listeners to set the
        // values now.
        MoPrefs.shoulderEncoderScale.subscribe(scale -> MoUtils.setupRelativeEncoder(
                shoulderRelEncoder, shoulderAbsEncoder.getPosition(), MoPrefs.shoulderAbsZero.get(), scale));
        MoPrefs.shoulderAbsEncoderScale.subscribe(shoulderAbsEncoder::setConversionFactor, true);
        MoPrefs.wristEncoderScale.subscribe(scale -> MoUtils.setupRelativeEncoder(
                wristRelEncoder, wristAbsEncoder.getPosition(), MoPrefs.wristAbsZero.get(), scale));
        MoPrefs.shoulderAbsZero.subscribe(zero -> MoUtils.setupRelativeEncoder(
                shoulderRelEncoder, shoulderAbsEncoder.getPosition(), zero, MoPrefs.shoulderEncoderScale.get()));
        MoPrefs.wristAbsZero.subscribe(zero -> MoUtils.setupRelativeEncoder(
                wristRelEncoder, wristAbsEncoder.getPosition(), zero, MoPrefs.wristEncoderScale.get()));

        shoulderLeftMtr.setSoftLimit(SoftLimitDirection.kReverse, 0);
        wristMtr.setSoftLimit(SoftLimitDirection.kReverse, 0);
        MoPrefs.shoulderMaxExtension.subscribe(limit -> shoulderLeftMtr.setSoftLimit(
                SoftLimitDirection.kForward, (float) limit.in(shoulderRelEncoder.getInternalEncoderUnits())));
        MoPrefs.wristMaxExtension.subscribe(limit -> wristMtr.setSoftLimit(
                SoftLimitDirection.kForward, (float) limit.in(wristRelEncoder.getInternalEncoderUnits())));

        shoulderLeftMtr.enableSoftLimit(SoftLimitDirection.kReverse, true);
        shoulderLeftMtr.enableSoftLimit(SoftLimitDirection.kForward, true);
        wristMtr.enableSoftLimit(SoftLimitDirection.kReverse, true);
        wristMtr.enableSoftLimit(SoftLimitDirection.kForward, true);

        shoulderVelocityPid = new MoSparkMaxArmPID(
                MoSparkMaxPID.Type.VELOCITY,
                shoulderLeftMtr,
                0,
                shoulderRelEncoder,
                this::getShoulderAngleFromHorizontal);
        wristVelocityPid = new MoSparkMaxArmPID(
                MoSparkMaxPID.Type.VELOCITY, wristMtr, 0, wristRelEncoder, this::getWristAngleFromHorizontal);
        shoulderSmartMotionPid = new MoSparkMaxArmPID(
                MoSparkMaxPID.Type.SMARTMOTION,
                shoulderLeftMtr,
                1,
                shoulderRelEncoder,
                this::getShoulderAngleFromHorizontal);
        wristSmartMotionPid = new MoSparkMaxArmPID(
                MoSparkMaxPID.Type.SMARTMOTION, wristMtr, 1, wristRelEncoder, this::getWristAngleFromHorizontal);

        shoulderVelTuner = TunerUtils.forSparkMaxArm(shoulderVelocityPid, "Shoulder Vel.");
        wristVelTuner = TunerUtils.forSparkMaxArm(wristVelocityPid, "Wrist Vel.");
        shoulderPosTuner = TunerUtils.forSparkMaxArm(shoulderSmartMotionPid, "Shoulder Pos.");
        wristPosTuner = TunerUtils.forSparkMaxArm(wristSmartMotionPid, "Wrist Pos.");

        var shoulderGroup = MoShuffleboard.getInstance()
                .armTab
                .getLayout("Shoulder Position", BuiltInLayouts.kList)
                .withSize(2, 1)
                .withProperties(Map.of("Label position", "RIGHT"));
        shoulderGroup.addDouble(
                "Relative", () -> shoulderRelEncoder.getPosition().in(Units.Rotations));
        shoulderGroup.addDouble(
                "Absolute", () -> shoulderAbsEncoder.getPosition().in(Units.Rotations));
        shoulderGroup.addDouble(
                "Rel Vel.", () -> shoulderRelEncoder.getVelocity().in(Units.RotationsPerSecond));

        var wristGroup = MoShuffleboard.getInstance()
                .armTab
                .getLayout("Wrist Position", BuiltInLayouts.kList)
                .withSize(2, 1)
                .withProperties(Map.of("Label position", "RIGHT"));
        wristGroup.addDouble("Relative", () -> wristRelEncoder.getPosition().in(Units.Rotations));
        wristGroup.addDouble("Absolute", () -> wristAbsEncoder.getPosition().in(Units.Rotations));
        wristGroup.addDouble("Rel Vel.", () -> wristRelEncoder.getVelocity().in(Units.RotationsPerSecond));

        controlMode = MoShuffleboard.enumToChooser(ArmControlMode.class);
        MoShuffleboard.getInstance().settingsTab.add("Arm Control Mode", controlMode);

        MoShuffleboard.getInstance().armTab.add(this);

        coastArmsEntry =
                MoShuffleboard.getInstance().armTab.add("Coast Arms", false).getEntry();

        configureMotors();
    }

    public void setShoulderReverseLimitEnabled(boolean enabled) {
        shoulderLeftMtr.enableSoftLimit(SoftLimitDirection.kReverse, enabled);
    }

    public void reZeroArm() {
        MoUtils.setupRelativeEncoder(
                shoulderRelEncoder,
                shoulderAbsEncoder.getPosition(),
                MoPrefs.shoulderAbsZero.get(),
                MoPrefs.shoulderEncoderScale.get());
        MoUtils.setupRelativeEncoder(
                wristRelEncoder,
                wristAbsEncoder.getPosition(),
                MoPrefs.wristAbsZero.get(),
                MoPrefs.wristEncoderScale.get());
    }

    private Measure<Angle> getShoulderAngleFromHorizontal() {
        return shoulderRelEncoder.getPosition().minus(MoPrefs.shoulderHorizontal.get());
    }

    private Measure<Angle> getWristAngleFromShoulder() {
        return wristRelEncoder.getPosition().plus(MoPrefs.wristZeroOffsetFromShoulder.get());
    }

    private static final Measure<Angle> half_rot = Units.Rotations.of(0.5);

    private Measure<Angle> getWristAngleFromHorizontal() {
        return half_rot.plus(getShoulderAngleFromHorizontal()).minus(getWristAngleFromShoulder());
    }

    private ArmMovementRequest limitArmMovementRequest(ArmMovementRequest request) {
        double shoulderPower = request.shoulderPower;
        double wristPower = request.wristPower;

        var shoulderPos = shoulderRelEncoder.getPosition();
        var wristPos = wristRelEncoder.getPosition();

        if (shoulderPower > 0 && shoulderPos.gt(MoPrefs.shoulderMaxExtension.get())) {
            shoulderPower = 0;
        }
        if (shoulderPower < 0 && shoulderPos.lt(Units.Rotations.zero())) {
            shoulderPower = 0;
        }
        if (wristPower > 0 && wristPos.gt(MoPrefs.wristMaxExtension.get())) {
            wristPower = 0;
        }
        if (wristPower < 0 && wristPos.lt(Units.Rotations.zero())) {
            wristPower = 0;
        }

        return new ArmMovementRequest(shoulderPower, wristPower);
    }

    public ArmPosition getArmPosition() {
        return new ArmPosition(shoulderRelEncoder.getPosition(), wristRelEncoder.getPosition());
    }

    public void adjustDirectPower(ArmMovementRequest request) {
        request = limitArmMovementRequest(request);
        shoulderLeftMtr.set(request.shoulderPower);
        wristMtr.set(request.wristPower);
    }

    public void adjustVelocity(ArmMovementRequest request) {
        request = limitArmMovementRequest(request);

        Measure<Velocity<Angle>> shoulderVelocity = MoPrefs.shoulderMaxRps.get().times(request.shoulderPower);
        Measure<Velocity<Angle>> wristVelocity = MoPrefs.wristMaxRps.get().times(request.wristPower);

        shoulderVelocityPid.setVelocityReference(shoulderVelocity);
        wristVelocityPid.setVelocityReference(wristVelocity);
    }

    public void adjustSmartPosition(ArmPosition position) {
        if (trustWristEncoder) {
            shoulderSmartMotionPid.setPositionReference(position.shoulderAngle);
        } else {
            shoulderVelocityPid.setVelocityReference(Units.RPM.zero());
        }
        wristSmartMotionPid.setPositionReference(position.wristAngle);
    }

    public boolean atPosition(ArmPosition position) {

        double thresh = MoPrefs.armSetpointVarianceThreshold.get().in(Units.Value);
        return shoulderRelEncoder.getPosition().isNear(position.shoulderAngle(), thresh)
                && wristRelEncoder.getPosition().isNear(position.wristAngle(), thresh);
    }

    public SysIdRoutine getShoulderRoutine() {
        var config = MoShuffleboard.getInstance().getSysidConfig();

        final MutableMeasure<Voltage> mut_volt = MutableMeasure.zero(Units.Volts);

        return new SysIdRoutine(
                config,
                new SysIdRoutine.Mechanism(
                        (v) -> {
                            shoulderLeftMtr.setVoltage(v.in(Units.Volts));
                            wristMtr.stopMotor();
                        },
                        (log) -> {
                            log.motor("shoulderLeftMtr")
                                    .voltage(mut_volt.mut_replace(
                                            shoulderLeftMtr.getAppliedOutput() * shoulderLeftMtr.getBusVoltage(),
                                            Units.Volts))
                                    .angularPosition(shoulderRelEncoder.getPosition())
                                    .angularVelocity(shoulderRelEncoder.getVelocity());
                        },
                        this));
    }

    public SysIdRoutine getWristRoutine() {
        var config = MoShuffleboard.getInstance().getSysidConfig();

        final MutableMeasure<Voltage> mut_volt = MutableMeasure.zero(Units.Volts);

        return new SysIdRoutine(
                config,
                new SysIdRoutine.Mechanism(
                        (v) -> {
                            wristMtr.setVoltage(v.in(Units.Volts));
                            shoulderSmartMotionPid.setPositionReference(MoPrefs.shoulderHorizontal.get());
                        },
                        (log) -> {
                            log.motor("wristMtr")
                                    .voltage(mut_volt.mut_replace(
                                            wristMtr.getAppliedOutput() * wristMtr.getBusVoltage(), Units.Volts))
                                    .angularPosition(getWristAngleFromHorizontal())
                                    .angularVelocity(wristRelEncoder.getVelocity());
                        },
                        this));
    }

    @Override
    public void periodic() {
        CANSparkBase motors[] = {shoulderLeftMtr, shoulderRightMtr, wristMtr};
        boolean hasBrownout = false;
        for (CANSparkBase motor : motors) {
            if (motor.getStickyFault(FaultID.kBrownout) || motor.getStickyFault(FaultID.kHasReset)) {
                hasBrownout = true;
                break;
            }
        }
        if (!trustWristEncoder) {
            boolean encoderValid =
                    Math.abs(wristAbsEncoder.getPosition().in(Units.Rotations)) >= WRIST_ENCODER_INVALID_RANGE;
            if (encoderValid) {
                if (trustWristEncoderTimer.hasElapsed(WRIST_ENCODER_VALID_PERIOD.in(Units.Seconds))) {
                    trustWristEncoder = true;
                    MoUtils.setupRelativeEncoder(
                            wristRelEncoder,
                            wristAbsEncoder.getPosition(),
                            MoPrefs.wristAbsZero.get(),
                            MoPrefs.wristEncoderScale.get());
                }
            } else {
                trustWristEncoderTimer.restart();
            }
        }

        if (hasBrownout) {
            DriverStation.reportWarning("Arm brownout!!", false);
            System.out.println("Arm Brownout!!");
            configureMotors();
            for (CANSparkBase motor : motors) {
                motor.clearFaults();
            }
        }

        boolean coastArms = coastArmsEntry.getBoolean(false);
        if (coastArms != lastCoastArms) {
            lastCoastArms = coastArms;
            IdleMode idleMode = lastCoastArms ? IdleMode.kCoast : IdleMode.kBrake;
            shoulderLeftMtr.setIdleMode(idleMode);
            shoulderRightMtr.setIdleMode(idleMode);
            wristMtr.setIdleMode(idleMode);
        }
    }
}
