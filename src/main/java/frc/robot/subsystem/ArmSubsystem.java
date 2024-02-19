package frc.robot.subsystem;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MoPrefs;
import frc.robot.util.MoSparkMaxPID;
import frc.robot.util.MoUtils;
import frc.robot.util.TunerUtils;

public class ArmSubsystem extends SubsystemBase {
    private static final Measure<Current> SHOULDER_CURRENT_LIMIT = Units.Amps.of(50);
    private static final Measure<Current> WRIST_CURRENT_LIMIT = Units.Amps.of(50);

    private final CANSparkMax shoulderLeftMtr;
    private final CANSparkMax shoulderRightMtr;
    private final CANSparkMax wristMtr;

    private final SparkAbsoluteEncoder shoulderAbsEncoder;
    private final SparkAbsoluteEncoder wristAbsEncoder;

    private final RelativeEncoder shoulderRelEncoder;
    private final RelativeEncoder wristRelEncoder;

    private final MoSparkMaxPID shoulderVelocityPid;
    private final MoSparkMaxPID wristVelocityPid;

    private final MoSparkMaxPID shoulderSmartMotionPid;
    private final MoSparkMaxPID wristSmartMotionPid;

    private final MutableMeasure<Angle> mut_shoulderAbsPosition = MutableMeasure.zero(Units.Rotations);
    private final MutableMeasure<Angle> mut_wristAbsPosition = MutableMeasure.zero(Units.Rotations);
    private final MutableMeasure<Angle> mut_shoulderRelPosition = MutableMeasure.zero(Units.Rotations);
    private final MutableMeasure<Angle> mut_wristRelPosition = MutableMeasure.zero(Units.Rotations);

    public static record ArmPosition(Measure<Angle> shoulderAngle, Measure<Angle> wristAngle) {}

    public static record ArmMovementRequest(double shoulderPower, double wristPower) {
        public ArmMovementRequest(double shoulderPower, double wristPower) {
            this.shoulderPower = MoUtils.clamp(shoulderPower, -1, 1);
            this.wristPower = MoUtils.clamp(wristPower, -1, 1);
        }
    }

    public ArmSubsystem() {
        shoulderLeftMtr = new CANSparkMax(Constants.SHOULDER_LEFT_MTR.address(), MotorType.kBrushless);
        shoulderRightMtr = new CANSparkMax(Constants.SHOULDER_RIGHT_MTR.address(), MotorType.kBrushless);
        wristMtr = new CANSparkMax(Constants.WRIST_MTR.address(), MotorType.kBrushless);

        shoulderLeftMtr.setSmartCurrentLimit((int) SHOULDER_CURRENT_LIMIT.in(Units.Amps));
        shoulderRightMtr.setSmartCurrentLimit((int) SHOULDER_CURRENT_LIMIT.in(Units.Amps));
        wristMtr.setSmartCurrentLimit((int) WRIST_CURRENT_LIMIT.in(Units.Amps));

        shoulderLeftMtr.setIdleMode(IdleMode.kBrake);
        shoulderRightMtr.setIdleMode(IdleMode.kBrake);
        wristMtr.setIdleMode(IdleMode.kBrake);

        shoulderLeftMtr.setInverted(false);
        shoulderRightMtr.follow(shoulderLeftMtr, true);

        // TODO: Ensure the shoulder abs encoder is wired to the left spark
        shoulderAbsEncoder = shoulderLeftMtr.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        wristAbsEncoder = wristMtr.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

        shoulderRelEncoder = shoulderLeftMtr.getEncoder();
        wristRelEncoder = wristMtr.getEncoder();

        // Setup listeners for encoder scales and absolute zeros. Use notifyImmediately on zero listeners to set the
        // values now.
        MoPrefs.shoulderEncoderScale.subscribe(scale -> MoUtils.setupRelativeEncoder(
                shoulderRelEncoder, getShoulderAbsPosition(), MoPrefs.shoulderAbsZero.get(), scale));
        MoPrefs.wristEncoderScale.subscribe(scale -> MoUtils.setupRelativeEncoder(
                wristRelEncoder, getWristAbsPosition(), MoPrefs.wristAbsZero.get(), scale));
        MoPrefs.shoulderAbsZero.subscribe(
                zero -> MoUtils.setupRelativeEncoder(
                        shoulderRelEncoder, getShoulderAbsPosition(), zero, MoPrefs.shoulderEncoderScale.get()),
                true);
        MoPrefs.wristAbsZero.subscribe(
                zero -> MoUtils.setupRelativeEncoder(
                        wristRelEncoder, getWristAbsPosition(), zero, MoPrefs.wristEncoderScale.get()),
                true);

        shoulderLeftMtr.setSoftLimit(SoftLimitDirection.kReverse, 0);
        wristMtr.setSoftLimit(SoftLimitDirection.kReverse, 0);
        MoPrefs.shoulderMaxExtension.subscribe(
                limit -> shoulderLeftMtr.setSoftLimit(SoftLimitDirection.kForward, (float) limit.in(Units.Rotations)),
                true);
        MoPrefs.wristMaxExtension.subscribe(
                limit -> wristMtr.setSoftLimit(SoftLimitDirection.kForward, (float) limit.in(Units.Rotations)), true);

        shoulderVelocityPid = new MoSparkMaxPID(MoSparkMaxPID.Type.VELOCITY, shoulderLeftMtr, 0);
        wristVelocityPid = new MoSparkMaxPID(MoSparkMaxPID.Type.VELOCITY, wristMtr, 0);
        shoulderSmartMotionPid = new MoSparkMaxPID(MoSparkMaxPID.Type.SMARTMOTION, shoulderLeftMtr, 1);
        wristSmartMotionPid = new MoSparkMaxPID(MoSparkMaxPID.Type.SMARTMOTION, wristMtr, 1);

        TunerUtils.forMoSparkMax(shoulderVelocityPid, "Shoulder Vel.");
        TunerUtils.forMoSparkMax(wristVelocityPid, "Wrist Vel.");
        TunerUtils.forMoSparkMax(shoulderSmartMotionPid, "Shoulder Pos.");
        TunerUtils.forMoSparkMax(wristSmartMotionPid, "Wrist Pos.");
    }

    private Measure<Angle> getShoulderAbsPosition() {
        return mut_shoulderAbsPosition.mut_replace(shoulderAbsEncoder.getPosition(), Units.Rotations);
    }

    private Measure<Angle> getWristAbsPosition() {
        return mut_wristAbsPosition.mut_replace(wristAbsEncoder.getPosition(), Units.Rotations);
    }

    private Measure<Angle> getShoulderRelPosition() {
        return mut_shoulderRelPosition.mut_replace(shoulderRelEncoder.getPosition(), Units.Rotations);
    }

    private Measure<Angle> getWristRelPosition() {
        return mut_wristRelPosition.mut_replace(wristRelEncoder.getPosition(), Units.Rotations);
    }

    private ArmMovementRequest limitArmMovementRequest(ArmMovementRequest request) {
        double shoulderPower = request.shoulderPower;
        double wristPower = request.wristPower;

        if (shoulderPower > 0 && getShoulderRelPosition().gt(MoPrefs.shoulderMaxExtension.get())) {
            shoulderPower = 0;
        }
        if (shoulderPower < 0 && getShoulderRelPosition().lt(Units.Rotations.zero())) {
            shoulderPower = 0;
        }
        if (wristPower > 0 && getWristRelPosition().gt(MoPrefs.wristMaxExtension.get())) {
            wristPower = 0;
        }
        if (wristPower < 0 && getWristRelPosition().lt(Units.Rotations.zero())) {
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
        return new ArmPosition(getShoulderRelPosition(), getWristRelPosition());
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

        shoulderVelocityPid.setReference(shoulderVelocity.in(Units.RotationsPerSecond));
        wristVelocityPid.setReference(wristVelocity.in(Units.RotationsPerSecond));
    }

    public void adjustSmartPosition(ArmPosition position) {
        position = limitArmPositionRequest(position);

        shoulderSmartMotionPid.setReference(position.shoulderAngle.in(Units.Rotations));
        wristSmartMotionPid.setReference(position.wristAngle.in(Units.Rotations));
    }
}
