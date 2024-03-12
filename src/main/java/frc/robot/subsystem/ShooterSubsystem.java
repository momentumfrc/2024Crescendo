package frc.robot.subsystem;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.encoder.MoEncoder;
import frc.robot.util.MoPrefs;
import frc.robot.util.MoShuffleboard;
import frc.robot.util.MoSparkMaxPID;
import frc.robot.util.MoSparkMaxPID.Type;
import frc.robot.util.MoUnits;
import frc.robot.util.TunerUtils;
import java.util.Map;

public class ShooterSubsystem extends SubsystemBase {
    private static final Measure<Current> ROLLER_CURRENT_LIMIT = Units.Amps.of(50);
    private static final Measure<Current> FLYWHEEL_CURRENT_LIMIT = Units.Amps.of(50);

    private final CANSparkMax roller;
    private final CANSparkFlex flywheelLower;
    private final CANSparkFlex flywheelUpper;

    private final MoEncoder<Distance> rollerEncoder;
    private final MoEncoder<Distance> flywheelUpperEncoder;
    private final MoEncoder<Distance> flywheelLowerEncoder;

    private final MoSparkMaxPID<Distance> rollerVelPid;
    private final MoSparkMaxPID<Distance> rollerPosPid;
    private final MoSparkMaxPID<Distance> flywheelUpperVelocityPid;
    private final MoSparkMaxPID<Distance> flywheelLowerVelocityPid;

    private final MutableMeasure<Distance> mut_flywheelPos = MutableMeasure.zero(Units.Centimeters);
    private final MutableMeasure<Velocity<Distance>> mut_flywheelVel = MutableMeasure.zero(MoUnits.CentimetersPerSec);

    public ShooterSubsystem() {
        super("Shooter");

        roller = new CANSparkMax(Constants.SHOOTER_ROLLER_MTR.address(), MotorType.kBrushless);
        flywheelUpper = new CANSparkFlex(Constants.SHOOTER_FLYWHEEL_MTR_UPPER.address(), MotorType.kBrushless);
        flywheelLower = new CANSparkFlex(Constants.SHOOTER_FLYWHEEL_MTR_LOWER.address(), MotorType.kBrushless);

        roller.setSmartCurrentLimit((int) ROLLER_CURRENT_LIMIT.in(Units.Amps));
        flywheelUpper.setSmartCurrentLimit((int) FLYWHEEL_CURRENT_LIMIT.in(Units.Amps));
        flywheelLower.setSmartCurrentLimit((int) FLYWHEEL_CURRENT_LIMIT.in(Units.Amps));

        roller.setIdleMode(IdleMode.kCoast);
        flywheelUpper.setIdleMode(IdleMode.kCoast);
        flywheelLower.setIdleMode(IdleMode.kCoast);

        flywheelUpper.setInverted(false);
        flywheelLower.setInverted(true);

        rollerEncoder = MoEncoder.forSparkRelative(roller.getEncoder(), Units.Centimeter);
        flywheelUpperEncoder = MoEncoder.forSparkRelative(flywheelUpper.getEncoder(), Units.Centimeters);
        flywheelLowerEncoder = MoEncoder.forSparkRelative(flywheelLower.getEncoder(), Units.Centimeters);

        MoPrefs.shooterRollerScale.subscribe(scale -> rollerEncoder.setConversionFactor(scale), true);
        MoPrefs.shooterFlywheelScale.subscribe(
                scale -> {
                    flywheelUpperEncoder.setConversionFactor(scale);
                    flywheelLowerEncoder.setConversionFactor(scale);
                },
                true);

        var shooterGroup = MoShuffleboard.getInstance()
                .matchTab
                .getLayout("Shooter", BuiltInLayouts.kList)
                .withSize(2, 1)
                .withProperties(Map.of("Label position", "LEFT"));
        shooterGroup.addDouble(
                "Roller Pos. (cm)", () -> rollerEncoder.getPosition().in(Units.Centimeters));
        shooterGroup.addDouble(
                "Flywheel Upper Vel. (cm_s)",
                () -> flywheelUpperEncoder.getVelocity().in(MoUnits.CentimetersPerSec));
        shooterGroup.addDouble(
                "Flywheel Lower Vel. (cm_s)",
                () -> flywheelLowerEncoder.getVelocity().in(MoUnits.CentimetersPerSec));

        rollerVelPid = new MoSparkMaxPID<>(Type.VELOCITY, roller, 1, rollerEncoder);
        rollerPosPid = new MoSparkMaxPID<>(Type.SMARTMOTION, roller, 0, rollerEncoder);
        TunerUtils.forMoSparkMax(rollerVelPid, "Shooter Roller Vel.");
        TunerUtils.forMoSparkMax(rollerPosPid, "Shooter Roller Pos.");

        flywheelUpperVelocityPid = new MoSparkMaxPID<>(Type.VELOCITY, flywheelUpper, 0, flywheelUpperEncoder);
        flywheelLowerVelocityPid = new MoSparkMaxPID<>(Type.VELOCITY, flywheelLower, 0, flywheelLowerEncoder);
        TunerUtils.forMoSparkMax(flywheelUpperVelocityPid, "Shooter Upper Flywheel Vel.");
        TunerUtils.forMoSparkMax(flywheelLowerVelocityPid, "Shooter Lower Flywheel Vel.");
    }

    public Measure<Distance> getRollerPosition() {
        return rollerEncoder.getPosition();
    }

    public Measure<Velocity<Distance>> getRollerVelocity() {
        return rollerEncoder.getVelocity();
    }

    public Measure<Distance> getAvgFlywheelPosition() {
        mut_flywheelPos.mut_replace(flywheelUpperEncoder.getPosition());
        mut_flywheelPos.mut_plus(flywheelLowerEncoder.getPosition());
        mut_flywheelPos.mut_divide(2);

        return mut_flywheelPos;
    }

    public Measure<Velocity<Distance>> getAvgFlywheelVelocity() {
        mut_flywheelVel.mut_replace(flywheelUpperEncoder.getVelocity());
        mut_flywheelVel.mut_plus(flywheelLowerEncoder.getVelocity());
        mut_flywheelVel.mut_divide(2);

        return mut_flywheelVel;
    }

    public void directDriveRoller(double speed) {
        roller.set(speed);
    }

    public void setRollerVelocity(Measure<Velocity<Distance>> vel) {
        rollerVelPid.setVelocityReference(vel);
    }

    public void setRollerPosition(Measure<Distance> pos) {
        rollerPosPid.setPositionReference(pos);
    }

    public void directDriveFlywheel(double speed) {
        flywheelUpper.set(speed);
        flywheelLower.set(speed);
    }

    public void setFlywheelSpeed(Measure<Velocity<Distance>> speed) {
        flywheelUpperVelocityPid.setVelocityReference(speed);
        flywheelLowerVelocityPid.setVelocityReference(speed);
    }

    public SysIdRoutine getFlywheelUpperRoutine(SysIdRoutine.Config config) {
        var voltsPerSec = Units.Volts.per(Units.Second);
        if (config == null) {
            config = new SysIdRoutine.Config(voltsPerSec.of(1.5), Units.Volts.of(4), Units.Seconds.of(45));
        }

        final MutableMeasure<Voltage> mut_volt = MutableMeasure.zero(Units.Volts);

        return new SysIdRoutine(
                config,
                new SysIdRoutine.Mechanism(
                        (v) -> {
                            flywheelUpper.setVoltage(v.in(Units.Volts));
                            flywheelLower.stopMotor();
                            roller.stopMotor();
                        },
                        (log) -> {
                            log.motor("flywheelUpperMtr")
                                    .voltage(mut_volt.mut_replace(
                                            flywheelUpper.getAppliedOutput() * flywheelUpper.getBusVoltage(),
                                            Units.Volts))
                                    .linearPosition(flywheelUpperEncoder.getPosition())
                                    .linearVelocity(flywheelUpperEncoder.getVelocity());
                        },
                        this));
    }

    public SysIdRoutine getFlywheelLowerRoutine(SysIdRoutine.Config config) {
        var voltsPerSec = Units.Volts.per(Units.Second);
        if (config == null) {
            config = new SysIdRoutine.Config(voltsPerSec.of(1.5), Units.Volts.of(4), Units.Seconds.of(45));
        }

        final MutableMeasure<Voltage> mut_volt = MutableMeasure.zero(Units.Volts);

        return new SysIdRoutine(
                config,
                new SysIdRoutine.Mechanism(
                        (v) -> {
                            flywheelLower.setVoltage(v.in(Units.Volts));
                            flywheelUpper.stopMotor();
                            roller.stopMotor();
                        },
                        (log) -> {
                            log.motor("flywheelLowerMtr")
                                    .voltage(mut_volt.mut_replace(
                                            flywheelLower.getAppliedOutput() * flywheelLower.getBusVoltage(),
                                            Units.Volts))
                                    .linearPosition(flywheelLowerEncoder.getPosition())
                                    .linearVelocity(flywheelLowerEncoder.getVelocity());
                        },
                        this));
    }

    public SysIdRoutine getRollerRoutine(SysIdRoutine.Config config) {
        var voltsPerSec = Units.Volts.per(Units.Second);
        if (config == null) {
            config = new SysIdRoutine.Config(voltsPerSec.of(1.5), Units.Volts.of(4), Units.Seconds.of(45));
        }

        final MutableMeasure<Voltage> mut_volt = MutableMeasure.zero(Units.Volts);

        return new SysIdRoutine(
                config,
                new SysIdRoutine.Mechanism(
                        (v) -> {
                            roller.setVoltage(v.in(Units.Volts));
                            flywheelUpper.stopMotor();
                            flywheelLower.stopMotor();
                        },
                        (log) -> {
                            log.motor("rollerMtr")
                                    .voltage(mut_volt.mut_replace(
                                            roller.getAppliedOutput() * roller.getBusVoltage(), Units.Volts))
                                    .linearPosition(rollerEncoder.getPosition())
                                    .linearVelocity(rollerEncoder.getVelocity());
                        },
                        this));
    }
}
