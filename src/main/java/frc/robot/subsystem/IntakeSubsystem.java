package frc.robot.subsystem;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.encoder.MoEncoder;
import frc.robot.util.MoPrefs;
import frc.robot.util.MoShuffleboard;
import frc.robot.util.MoSparkMaxPID;
import frc.robot.util.MoUtils;
import frc.robot.util.TunerUtils;
import java.util.Map;

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

    private final MoEncoder<Distance> rollerEncoder;
    private final MoEncoder<Angle> deployEncoder;
    private final MoEncoder<Angle> deployAbsoluteEncoder;

    private final MoSparkMaxPID<Distance> rollerPID;
    private final MoSparkMaxPID<Angle> deployVelocityPID;
    private final MoSparkMaxPID<Angle> deploySmartmotionPID;

    public final SendableChooser<IntakeControlMode> controlMode;

    public IntakeSubsystem() {
        rollerMtr = new CANSparkMax(Constants.INTAKE_ROLLER.address(), MotorType.kBrushless);
        deployMtr = new CANSparkMax(Constants.INTAKE_DEPLOY.address(), MotorType.kBrushless);

        rollerMtr.setSmartCurrentLimit((int) ROLLER_CURRENT_LIMIT.in(Units.Amps));
        deployMtr.setSmartCurrentLimit((int) DEPLOY_CURRENT_LIMIT.in(Units.Amps));

        rollerMtr.setIdleMode(IdleMode.kCoast);
        deployMtr.setIdleMode(IdleMode.kCoast);

        rollerEncoder = MoEncoder.forSparkRelative(rollerMtr.getEncoder(), Units.Centimeters);
        deployEncoder = MoEncoder.forSparkRelative(deployMtr.getEncoder(), Units.Rotations);
        deployAbsoluteEncoder = MoEncoder.forSparkAbsolute(
                deployMtr.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle), Units.Rotations);

        MoPrefs.intakeRollerScale.subscribe(scale -> rollerEncoder.setConversionFactor(scale), true);
        MoPrefs.intakeDeployScale.subscribe(
                scale -> MoUtils.setupRelativeEncoder(
                        deployEncoder, deployAbsoluteEncoder.getPosition(), MoPrefs.intakeDeployAbsZero.get(), scale),
                false);
        MoPrefs.intakeDeployAbsZero.subscribe(
                zero -> MoUtils.setupRelativeEncoder(
                        deployEncoder, deployAbsoluteEncoder.getPosition(), zero, MoPrefs.intakeDeployScale.get()),
                true);

        deployMtr.setSoftLimit(SoftLimitDirection.kReverse, 0);
        deployMtr.setSoftLimit(SoftLimitDirection.kForward, (float)
                deployEncoder.positionInEncoderUnits(MoPrefs.intakeDeployMaxExtension.get()));

        deployMtr.enableSoftLimit(SoftLimitDirection.kReverse, true);
        deployMtr.enableSoftLimit(SoftLimitDirection.kForward, true);

        rollerPID = new MoSparkMaxPID<>(MoSparkMaxPID.Type.SMARTMOTION, rollerMtr, 0, rollerEncoder);
        deployVelocityPID = new MoSparkMaxPID<>(MoSparkMaxPID.Type.VELOCITY, deployMtr, 0, deployEncoder);
        deploySmartmotionPID = new MoSparkMaxPID<>(MoSparkMaxPID.Type.SMARTMOTION, deployMtr, 1, deployEncoder);

        TunerUtils.forMoSparkMax(rollerPID, "Intake Roller Pos.");
        TunerUtils.forMoSparkMax(deployVelocityPID, "Intake Deploy Vel.");
        TunerUtils.forMoSparkMax(deploySmartmotionPID, "Intake Deploy Pos.");

        var deployGroup = MoShuffleboard.getInstance()
                .matchTab
                .getLayout("Intake Position", BuiltInLayouts.kList)
                .withSize(2, 1)
                .withProperties(Map.of("Label Position", "RIGHT"));
        deployGroup.addDouble("Relative", () -> deployEncoder.getPosition().in(Units.Rotations));
        deployGroup.addDouble(
                "Absolute", () -> deployAbsoluteEncoder.getPosition().in(Units.Rotations));
        deployGroup.addDouble("Rel Vel.", () -> deployEncoder.getVelocity().in(Units.RotationsPerSecond));

        controlMode = MoShuffleboard.enumToChooser(IntakeControlMode.class);
        MoShuffleboard.getInstance().settingsTab.add("Intake Control Mode", controlMode);
    }

    public Measure<Angle> getDeployPosition() {
        return deployEncoder.getPosition();
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
}
