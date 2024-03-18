package frc.robot.subsystem;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.encoder.MoEncoder;
import frc.robot.util.MoPrefs;
import frc.robot.util.MoShuffleboard;
import frc.robot.util.MoSparkMaxArmPID;
import frc.robot.util.MoSparkMaxPID;
import frc.robot.util.TunerUtils;
import java.util.Map;
import java.util.function.Supplier;

public class IntakeSubsystem extends SubsystemBase {
    private static final Measure<Current> ROLLER_CURRENT_LIMIT = Units.Amps.of(30);
    private static final Measure<Current> DEPLOY_CURRENT_LIMIT = Units.Amps.of(40);

    public static enum IntakeControlMode {
        SMARTMOTION,
        DIRECT_VELOCITY,
        FALLBACK_DIRECT_POWER
    };

    private final CANSparkMax rollerMtr;
    private final CANSparkMax deployMtr;

    private final MoEncoder<Angle> deployEncoder;

    private final MoSparkMaxArmPID deployVelocityPID;
    private final MoSparkMaxArmPID deploySmartmotionPID;

    public final GenericEntry isDeployZeroed;
    private final GenericEntry isHoldingNote;

    public final GenericPublisher setpointPublisher;

    public final SendableChooser<IntakeControlMode> controlMode;

    private final MutableMeasure<Current> mut_intakeRollerCurrent = MutableMeasure.zero(Units.Amps);
    private final MutableMeasure<Current> mut_deployMtrCurrent = MutableMeasure.zero(Units.Amps);

    public IntakeSubsystem() {
        super("Intake");

        rollerMtr = new CANSparkMax(Constants.INTAKE_ROLLER.address(), MotorType.kBrushed);
        deployMtr = new CANSparkMax(Constants.INTAKE_DEPLOY.address(), MotorType.kBrushless);

        rollerMtr.restoreFactoryDefaults();
        deployMtr.restoreFactoryDefaults();

        rollerMtr.setSmartCurrentLimit((int) ROLLER_CURRENT_LIMIT.in(Units.Amps));
        deployMtr.setSmartCurrentLimit((int) DEPLOY_CURRENT_LIMIT.in(Units.Amps));

        rollerMtr.setIdleMode(IdleMode.kCoast);
        deployMtr.setIdleMode(IdleMode.kBrake);

        rollerMtr.setInverted(true);
        deployMtr.setInverted(false);

        deployEncoder = MoEncoder.forSparkRelative(deployMtr.getEncoder(), Units.Rotations);

        MoPrefs.intakeDeployScale.subscribe(scale -> deployEncoder.setConversionFactor(scale), true);

        deployMtr.setSoftLimit(SoftLimitDirection.kReverse, 0);
        MoPrefs.intakeDeployMaxExtension.subscribe(
                max -> deployMtr.setSoftLimit(
                        SoftLimitDirection.kForward, (float) max.in(deployEncoder.getInternalEncoderUnits())),
                true);

        deployMtr.enableSoftLimit(SoftLimitDirection.kReverse, true);
        deployMtr.enableSoftLimit(SoftLimitDirection.kForward, true);

        final MutableMeasure<Angle> mut_angle = MutableMeasure.zero(Units.Rotations);
        Supplier<Measure<Angle>> intakeHorizontalAngle = () -> {
            mut_angle.mut_replace(deployEncoder.getPosition());
            return mut_angle.mut_minus(MoPrefs.intakeHorizontal.get());
        };

        deployVelocityPID =
                new MoSparkMaxArmPID(MoSparkMaxPID.Type.VELOCITY, deployMtr, 0, deployEncoder, intakeHorizontalAngle);
        deploySmartmotionPID = new MoSparkMaxArmPID(
                MoSparkMaxPID.Type.SMARTMOTION, deployMtr, 1, deployEncoder, intakeHorizontalAngle);

        TunerUtils.forSparkMaxArm(deployVelocityPID, "Intake Deploy Vel.");
        TunerUtils.forSparkMaxArm(deploySmartmotionPID, "Intake Deploy Pos.");

        var deployGroup = MoShuffleboard.getInstance()
                .intakeTab
                .getLayout("Intake Position", BuiltInLayouts.kList)
                .withSize(2, 1)
                .withProperties(Map.of("Label Position", "RIGHT"));
        deployGroup.addDouble("Relative", () -> deployEncoder.getPosition().in(Units.Rotations));
        deployGroup.addDouble("Rel Vel.", () -> deployEncoder.getVelocity().in(Units.RotationsPerSecond));

        controlMode = MoShuffleboard.enumToChooser(IntakeControlMode.class);
        MoShuffleboard.getInstance().settingsTab.add("Intake Control Mode", controlMode);

        setpointPublisher = MoShuffleboard.getInstance()
                .intakeTab
                .add("Setpoint", "UNKNOWN")
                .getEntry();

        isHoldingNote = MoShuffleboard.getInstance()
                .intakeTab
                .add("Intake Has Note", false)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .getEntry();

        isDeployZeroed = MoShuffleboard.getInstance()
                .intakeTab
                .add("Intake Zeroed", false)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .getEntry();

        var group = MoShuffleboard.getInstance()
                .intakeTab
                .getLayout("Intake Current", BuiltInLayouts.kList)
                .withSize(2, 1)
                .withProperties(Map.of("Label position", "RIGHT"));
        group.addDouble("Roller (A)", rollerMtr::getOutputCurrent);
        group.addDouble("Deploy (A)", deployMtr::getOutputCurrent);

        MoShuffleboard.getInstance().intakeTab.add(this);
    }

    public void enableDeploySoftLimitReverse(boolean enable) {
        deployMtr.enableSoftLimit(SoftLimitDirection.kReverse, enable);
    }

    public boolean getIsHoldingNote() {
        return isHoldingNote.getBoolean(false);
    }

    public void setIsHoldingNote(boolean value) {
        this.isHoldingNote.setBoolean(value);
    }

    public Measure<Angle> getDeployPosition() {
        return deployEncoder.getPosition();
    }

    public Measure<Current> getRollerCurrent() {
        return mut_intakeRollerCurrent.mut_replace(rollerMtr.getOutputCurrent(), Units.Amps);
    }

    public Measure<Current> getDeployCurrent() {
        return mut_deployMtrCurrent.mut_replace(deployMtr.getOutputCurrent(), Units.Amps);
    }

    public void deployFallbackDirectPower(double power) {
        deployMtr.set(power);
    }

    public void deployVelocity(Measure<Velocity<Angle>> velocity) {
        deployVelocityPID.setVelocityReference(velocity);
    }

    public void deploySmartMotion(Measure<Angle> position) {
        deploySmartmotionPID.setPositionReference(position);
    }

    public void rollerIntakeDirectPower(double power) {
        rollerMtr.set(power);
    }

    public void zeroDeployEncoder(Measure<Angle> pos) {
        this.deployEncoder.setPosition(pos);
    }

    public SysIdRoutine getDeployRoutine() {
        var config = MoShuffleboard.getInstance().getSysidConfig();

        final MutableMeasure<Voltage> mut_volt = MutableMeasure.zero(Units.Volts);

        return new SysIdRoutine(
                config,
                new SysIdRoutine.Mechanism(
                        (v) -> {
                            deployMtr.setVoltage(v.in(Units.Volts));
                            rollerMtr.stopMotor();
                        },
                        (log) -> {
                            log.motor("intakeDeployMtr")
                                    .voltage(mut_volt.mut_replace(
                                            deployMtr.getAppliedOutput() * deployMtr.getBusVoltage(), Units.Volts))
                                    .angularPosition(deployEncoder.getPosition())
                                    .angularVelocity(deployEncoder.getVelocity());
                        },
                        this));
    }
}
