package frc.robot.subsystem;

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
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
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
import java.util.function.Supplier;

public class ArmSubsystem extends SubsystemBase {
    private static final Measure<Current> SHOULDER_CURRENT_LIMIT = Units.Amps.of(50);
    private static final Measure<Current> WRIST_CURRENT_LIMIT = Units.Amps.of(50);

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
    private final MoSparkMaxPID<Angle> wristVelocityPid;

    private final MoSparkMaxArmPID shoulderSmartMotionPid;
    private final MoSparkMaxPID<Angle> wristSmartMotionPid;

    public final SendableChooser<ArmControlMode> controlMode;

    private final GenericEntry voltRampEntry;
    private final GenericEntry voltEntry;

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

    public ArmSubsystem() {
        super("Arm");
        shoulderLeftMtr = new CANSparkMax(Constants.SHOULDER_LEFT_MTR.address(), MotorType.kBrushless);
        shoulderRightMtr = new CANSparkMax(Constants.SHOULDER_RIGHT_MTR.address(), MotorType.kBrushless);
        wristMtr = new CANSparkMax(Constants.WRIST_MTR.address(), MotorType.kBrushless);

        shoulderLeftMtr.setSmartCurrentLimit((int) SHOULDER_CURRENT_LIMIT.in(Units.Amps));
        shoulderRightMtr.setSmartCurrentLimit((int) SHOULDER_CURRENT_LIMIT.in(Units.Amps));
        wristMtr.setSmartCurrentLimit((int) WRIST_CURRENT_LIMIT.in(Units.Amps));

        shoulderLeftMtr.setIdleMode(IdleMode.kBrake);
        shoulderRightMtr.setIdleMode(IdleMode.kBrake);
        wristMtr.setIdleMode(IdleMode.kBrake);

        wristMtr.setInverted(true);

        shoulderLeftMtr.setInverted(true);
        shoulderRightMtr.follow(shoulderLeftMtr, true);

        // TODO: Ensure the shoulder abs encoder is wired to the left spark
        shoulderAbsEncoder = MoEncoder.forSparkAbsolute(
                shoulderLeftMtr.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle), Units.Rotations);

        var rawWristAbsEncoder = wristMtr.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        rawWristAbsEncoder.setInverted(true);
        wristAbsEncoder = MoEncoder.forSparkAbsolute(rawWristAbsEncoder, Units.Rotations);

        shoulderRelEncoder = MoEncoder.forSparkRelative(shoulderLeftMtr.getEncoder(), Units.Rotations);
        wristRelEncoder = MoEncoder.forSparkRelative(wristMtr.getEncoder(), Units.Rotations);

        // Setup listeners for encoder scales and absolute zeros. Use notifyImmediately on zero listeners to set the
        // values now.
        MoPrefs.shoulderEncoderScale.subscribe(scale -> MoUtils.setupRelativeEncoder(
                shoulderRelEncoder, shoulderAbsEncoder.getPosition(), MoPrefs.shoulderAbsZero.get(), scale));
        MoPrefs.shoulderAbsEncoderScale.subscribe(shoulderAbsEncoder::setConversionFactor, true);
        MoPrefs.wristEncoderScale.subscribe(scale -> MoUtils.setupRelativeEncoder(
                wristRelEncoder, wristAbsEncoder.getPosition(), MoPrefs.wristAbsZero.get(), scale));
        MoPrefs.shoulderAbsZero.subscribe(
                zero -> MoUtils.setupRelativeEncoder(
                        shoulderRelEncoder, shoulderAbsEncoder.getPosition(), zero, MoPrefs.shoulderEncoderScale.get()),
                true);
        MoPrefs.wristAbsZero.subscribe(
                zero -> MoUtils.setupRelativeEncoder(
                        wristRelEncoder, wristAbsEncoder.getPosition(), zero, MoPrefs.wristEncoderScale.get()),
                true);

        shoulderLeftMtr.setSoftLimit(SoftLimitDirection.kReverse, 0);
        wristMtr.setSoftLimit(SoftLimitDirection.kReverse, 0);
        MoPrefs.shoulderMaxExtension.subscribe(
                limit -> shoulderLeftMtr.setSoftLimit(SoftLimitDirection.kForward, (float) limit.in(Units.Rotations)),
                true);
        MoPrefs.wristMaxExtension.subscribe(
                limit -> wristMtr.setSoftLimit(SoftLimitDirection.kForward, (float) limit.in(Units.Rotations)), true);

        shoulderLeftMtr.enableSoftLimit(SoftLimitDirection.kReverse, true);
        shoulderLeftMtr.enableSoftLimit(SoftLimitDirection.kForward, true);
        wristMtr.enableSoftLimit(SoftLimitDirection.kReverse, true);
        wristMtr.enableSoftLimit(SoftLimitDirection.kForward, true);

        Supplier<Measure<Angle>> shoulderPosFromHorizontal =
                () -> shoulderRelEncoder.getPosition().minus(MoPrefs.shoulderHorizontal.get());
        shoulderVelocityPid = new MoSparkMaxArmPID(
                MoSparkMaxPID.Type.VELOCITY, shoulderLeftMtr, 0, shoulderRelEncoder, shoulderPosFromHorizontal);
        wristVelocityPid = new MoSparkMaxPID<Angle>(MoSparkMaxPID.Type.VELOCITY, wristMtr, 0, wristRelEncoder);
        shoulderSmartMotionPid = new MoSparkMaxArmPID(
                MoSparkMaxPID.Type.SMARTMOTION, shoulderLeftMtr, 1, shoulderRelEncoder, shoulderPosFromHorizontal);
        wristSmartMotionPid = new MoSparkMaxPID<Angle>(MoSparkMaxPID.Type.SMARTMOTION, wristMtr, 1, wristRelEncoder);

        TunerUtils.forSparkMaxArm(shoulderVelocityPid, "Shoulder Vel.");
        TunerUtils.forMoSparkMax(wristVelocityPid, "Wrist Vel.");
        TunerUtils.forSparkMaxArm(shoulderSmartMotionPid, "Shoulder Pos.");
        TunerUtils.forMoSparkMax(wristSmartMotionPid, "Wrist Pos.");

        var shoulderGroup = MoShuffleboard.getInstance()
                .matchTab
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
                .matchTab
                .getLayout("Wrist Position", BuiltInLayouts.kList)
                .withSize(2, 1)
                .withProperties(Map.of("Label position", "RIGHT"));
        wristGroup.addDouble("Relative", () -> wristRelEncoder.getPosition().in(Units.Rotations));
        wristGroup.addDouble("Absolute", () -> wristAbsEncoder.getPosition().in(Units.Rotations));
        wristGroup.addDouble("Rel Vel.", () -> wristRelEncoder.getVelocity().in(Units.RotationsPerSecond));

        var sysidGroup = MoShuffleboard.getInstance()
                .settingsTab
                .getLayout("Sysid Settings", BuiltInLayouts.kList)
                .withSize(2, 1)
                .withProperties(Map.of("Label position", "RIGHT"));
        voltRampEntry = sysidGroup.add("Volts Ramp", 1.5).getEntry();
        voltEntry = sysidGroup.add("Volts Step", 2).getEntry();

        controlMode = MoShuffleboard.enumToChooser(ArmControlMode.class);
        MoShuffleboard.getInstance().settingsTab.add("Arm Control Mode", controlMode);
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

    private ArmPosition limitArmPositionRequest(ArmPosition request) {
        return new ArmPosition(
                MoUtils.clampUnit(request.shoulderAngle, Units.Rotations.zero(), MoPrefs.shoulderMaxExtension.get()),
                MoUtils.clampUnit(request.wristAngle, Units.Rotations.zero(), MoPrefs.wristMaxExtension.get()));
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
        position = limitArmPositionRequest(position);

        shoulderSmartMotionPid.setPositionReference(position.shoulderAngle);
        wristSmartMotionPid.setPositionReference(position.wristAngle);
    }

    public boolean atPosition(ArmPosition position, double thresh) {
        return shoulderRelEncoder.getPosition().isNear(position.shoulderAngle, thresh)
                && wristRelEncoder.getPosition().isNear(position.wristAngle, thresh);
    }

    public SysIdRoutine getShoulderRoutine(SysIdRoutine.Config config) {
        var voltsPerSec = Units.Volts.per(Units.Second);
        if (config == null) {
            config = new SysIdRoutine.Config(
                    voltsPerSec.of(voltRampEntry.getDouble(1.5)),
                    Units.Volts.of(voltEntry.getDouble(3)),
                    Units.Seconds.of(45));
        }

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
}
