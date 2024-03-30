package frc.robot.subsystem;

import com.momentum4999.motune.PIDTuner;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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

public class ShooterSubsystem extends SubsystemBase {
    private static final Measure<Current> ROLLER_CURRENT_LIMIT = Units.Amps.of(50);
    private static final Measure<Current> FLYWHEEL_CURRENT_LIMIT = Units.Amps.of(50);

    public final CANSparkMax roller;
    private final CANSparkFlex flywheelLower;
    private final CANSparkFlex flywheelUpper;

    private final MoEncoder<Distance> rollerEncoder;
    private final MoEncoder<Distance> flywheelUpperEncoder;
    private final MoEncoder<Distance> flywheelLowerEncoder;

    private final MoSparkMaxPID<Distance> rollerVelPid;
    private final MoSparkMaxPID<Distance> rollerPosPid;
    private final MoSparkMaxPID<Distance> flywheelUpperVelocityPid;
    private final MoSparkMaxPID<Distance> flywheelLowerVelocityPid;

    private final PIDTuner rollerVelTuner;
    private final PIDTuner rollerPosTuner;
    private final PIDTuner flywheelUpperVelTuner;
    private final PIDTuner flywheelLowerVelTuner;

    private final MutableMeasure<Distance> mut_flywheelPos = MutableMeasure.zero(Units.Centimeters);
    private final MutableMeasure<Velocity<Distance>> mut_flywheelVel = MutableMeasure.zero(MoUnits.CentimetersPerSec);

    private final MutableMeasure<Velocity<Distance>> mut_flywheelSetpoint = MutableMeasure.zero(Units.MetersPerSecond);

    private final MutableMeasure<Current> mut_rollerCurrent = MutableMeasure.zero(Units.Amps);

    private SlewRateLimiter limiter;
    private SlewRateLimiter reverseLimiter;

    private Timer burnFlashDebounce = new Timer();

    public void configureMotors() {
        System.out.println("CONFIGURE SHOOTER MOTORS");
        roller.setSmartCurrentLimit((int) ROLLER_CURRENT_LIMIT.in(Units.Amps));
        flywheelUpper.setSmartCurrentLimit((int) FLYWHEEL_CURRENT_LIMIT.in(Units.Amps));
        flywheelLower.setSmartCurrentLimit((int) FLYWHEEL_CURRENT_LIMIT.in(Units.Amps));

        roller.setIdleMode(IdleMode.kCoast);
        flywheelUpper.setIdleMode(IdleMode.kCoast);
        flywheelLower.setIdleMode(IdleMode.kCoast);

        roller.setInverted(true);
        flywheelUpper.setInverted(false);
        flywheelLower.setInverted(false);

        rollerEncoder.setConversionFactor(MoPrefs.shooterRollerScale.get());
        flywheelUpperEncoder.setConversionFactor(MoPrefs.shooterFlywheelScale.get());
        flywheelLowerEncoder.setConversionFactor(MoPrefs.shooterFlywheelScale.get());

        rollerPosTuner.populatePIDValues();
        rollerVelTuner.populatePIDValues();
        flywheelLowerVelTuner.populatePIDValues();
        flywheelUpperVelTuner.populatePIDValues();
    }

    public ShooterSubsystem() {
        super("Shooter");

        roller = new CANSparkMax(Constants.SHOOTER_ROLLER_MTR.address(), MotorType.kBrushless);
        flywheelUpper = new CANSparkFlex(Constants.SHOOTER_FLYWHEEL_MTR_UPPER.address(), MotorType.kBrushless);
        flywheelLower = new CANSparkFlex(Constants.SHOOTER_FLYWHEEL_MTR_LOWER.address(), MotorType.kBrushless);

        rollerEncoder = MoEncoder.forSparkRelative(roller.getEncoder(), Units.Centimeter);
        flywheelUpperEncoder = MoEncoder.forSparkRelative(flywheelUpper.getEncoder(), Units.Meter);
        flywheelLowerEncoder = MoEncoder.forSparkRelative(flywheelLower.getEncoder(), Units.Meter);

        burnFlashDebounce.restart();

        MoPrefs.shooterRollerScale.subscribe(scale -> rollerEncoder.setConversionFactor(scale), true);
        MoPrefs.shooterFlywheelScale.subscribe(scale -> {
            flywheelUpperEncoder.setConversionFactor(scale);
            flywheelLowerEncoder.setConversionFactor(scale);
        });

        MoPrefs.flywheelSpindownRate.subscribe(
                rate -> {
                    this.limiter = new SlewRateLimiter(Double.POSITIVE_INFINITY, -Math.abs(rate), 0);
                    this.reverseLimiter = new SlewRateLimiter(Math.abs(rate), Double.NEGATIVE_INFINITY, 0);
                },
                true);

        MoShuffleboard.getInstance()
                .shooterTab
                .addDouble("Roller Pos. (cm)", () -> rollerEncoder.getPosition().in(Units.Centimeters));
        MoShuffleboard.getInstance().shooterTab.addDouble("Roller Vel. (cm_s)", () -> rollerEncoder
                .getVelocity()
                .in(MoUnits.CentimetersPerSec));
        MoShuffleboard.getInstance().shooterTab.addDouble("Flywheel Upper Vel. (m_s)", () -> flywheelUpperEncoder
                .getVelocity()
                .in(Units.MetersPerSecond));
        MoShuffleboard.getInstance().shooterTab.addDouble("Flywheel Lower Vel. (m_s)", () -> flywheelLowerEncoder
                .getVelocity()
                .in(Units.MetersPerSecond));

        MoShuffleboard.getInstance().shooterTab.addDouble("Flywheel Upper Pos. (m)", () -> flywheelUpperEncoder
                .getPosition()
                .in(Units.Meters));
        MoShuffleboard.getInstance().shooterTab.addDouble("Flywheel Lower Pos. (m)", () -> flywheelLowerEncoder
                .getPosition()
                .in(Units.Meters));

        rollerVelPid = new MoSparkMaxPID<>(Type.VELOCITY, roller, 1, rollerEncoder);
        rollerPosPid = new MoSparkMaxPID<>(Type.SMARTMOTION, roller, 0, rollerEncoder);
        rollerVelTuner = TunerUtils.forMoSparkMax(rollerVelPid, "Shooter Roller Vel.");
        rollerPosTuner = TunerUtils.forMoSparkMax(rollerPosPid, "Shooter Roller Pos.");

        flywheelUpperVelocityPid = new MoSparkMaxPID<>(Type.VELOCITY, flywheelUpper, 0, flywheelUpperEncoder);
        flywheelLowerVelocityPid = new MoSparkMaxPID<>(Type.VELOCITY, flywheelLower, 0, flywheelLowerEncoder);
        flywheelUpperVelTuner = TunerUtils.forMoSparkMax(flywheelUpperVelocityPid, "Shooter Upper Flywheel Vel.");
        flywheelLowerVelTuner = TunerUtils.forMoSparkMax(flywheelLowerVelocityPid, "Shooter Lower Flywheel Vel.");

        MoShuffleboard.getInstance().shooterTab.add(this);

        configureMotors();
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

    private Measure<Velocity<Distance>> slewLimitVelocity(Measure<Velocity<Distance>> speed) {
        SlewRateLimiter limiter;
        if (getAvgFlywheelVelocity().lte(Units.MetersPerSecond.zero())) {
            limiter = this.reverseLimiter;
        } else {
            limiter = this.limiter;
        }

        double value = speed.in(Units.MetersPerSecond);
        double limited = limiter.calculate(value);

        return mut_flywheelSetpoint.mut_replace(limited, Units.MetersPerSecond);
    }

    public void setFlywheelSpeed(Measure<Velocity<Distance>> speed) {
        var limited = slewLimitVelocity(speed);

        flywheelUpperVelocityPid.setVelocityReference(mut_flywheelSetpoint);
        flywheelLowerVelocityPid.setVelocityReference(mut_flywheelSetpoint);
    }

    public Measure<Current> getRollerCurrent() {
        return mut_rollerCurrent.mut_replace(roller.getOutputCurrent(), Units.Amps);
    }

    public SysIdRoutine getFlywheelUpperRoutine() {
        var config = MoShuffleboard.getInstance().getSysidConfig();

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

    public void burnFlash() {
        if (!burnFlashDebounce.hasElapsed(10)) {
            System.out.println("SHOULD NOT BURN FLASH THIS FREQUENLTY");
            return;
        }
        var rFlyLow = flywheelLower.burnFlash();
        var rFlyUp = flywheelUpper.burnFlash();
        var rRoll = roller.burnFlash();

        System.out.printf("flywL:%s, flywU:%s, roll%s", rFlyLow, rFlyUp, rRoll);
        System.out.println();

        burnFlashDebounce.restart();
    }

    public SysIdRoutine getFlywheelLowerRoutine() {
        var config = MoShuffleboard.getInstance().getSysidConfig();

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

    public SysIdRoutine getRollerRoutine() {
        var config = MoShuffleboard.getInstance().getSysidConfig();

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

    @Override
    public void periodic() {
        CANSparkBase motors[] = {flywheelUpper, flywheelLower, roller};
        boolean hasBrownout = false;
        for (CANSparkBase motor : motors) {
            if (motor.getStickyFault(FaultID.kBrownout) || motor.getStickyFault(FaultID.kHasReset)) {
                hasBrownout = true;
                break;
            }
        }

        if (hasBrownout) {
            DriverStation.reportWarning("Shooter brownout!!", false);
            System.out.println("Shooter Brownout!!");
            configureMotors();
            for (CANSparkBase motor : motors) {
                motor.clearFaults();
            }
        }
    }
}
