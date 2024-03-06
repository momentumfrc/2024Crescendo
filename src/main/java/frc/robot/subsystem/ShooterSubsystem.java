package frc.robot.subsystem;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.encoder.MoEncoder;
import frc.robot.util.MoPrefs;
import frc.robot.util.MoShuffleboard;
import frc.robot.util.MoSparkMaxPID;
import frc.robot.util.MoSparkMaxPID.Type;
import frc.robot.util.TunerUtils;
import java.util.Map;

public class ShooterSubsystem extends SubsystemBase {
    private static final Measure<Current> ROLLER_CURRENT_LIMIT = Units.Amps.of(50);
    private static final Measure<Current> FLYWHEEL_CURRENT_LIMIT = Units.Amps.of(50);

    private final CANSparkMax roller;
    private final CANSparkFlex flywheelLeft;
    private final CANSparkFlex flywheelRight;

    public final MoEncoder<Distance> rollerEncoder;
    public final MoEncoder<Angle> flywheelEncoder;

    private final MoSparkMaxPID<Distance> rollerPosPid;
    private final MoSparkMaxPID<Angle> flywheelVelocityPid;

    public ShooterSubsystem() {
        super("Shooter");

        roller = new CANSparkMax(Constants.SHOOTER_ROLLER_MTR.address(), MotorType.kBrushless);
        flywheelLeft = new CANSparkFlex(Constants.SHOOTER_FLYWHEEL_MTR_LEFT.address(), MotorType.kBrushless);
        flywheelRight = new CANSparkFlex(Constants.SHOOTER_FLYWHEEL_MTR_RIGHT.address(), MotorType.kBrushless);

        roller.setSmartCurrentLimit((int) ROLLER_CURRENT_LIMIT.in(Units.Amps));
        flywheelLeft.setSmartCurrentLimit((int) FLYWHEEL_CURRENT_LIMIT.in(Units.Amps));
        flywheelRight.setSmartCurrentLimit((int) FLYWHEEL_CURRENT_LIMIT.in(Units.Amps));

        roller.setIdleMode(IdleMode.kCoast);
        flywheelLeft.setIdleMode(IdleMode.kCoast);
        flywheelRight.setIdleMode(IdleMode.kCoast);

        flywheelLeft.setInverted(false);
        flywheelRight.follow(flywheelLeft, true);

        rollerEncoder = MoEncoder.forSparkRelative(roller.getEncoder(), Units.Centimeter);
        flywheelEncoder = MoEncoder.forSparkRelative(flywheelLeft.getEncoder(), Units.Rotations);

        MoPrefs.shooterRollerScale.subscribe(scale -> rollerEncoder.setConversionFactor(scale), true);
        MoPrefs.shooterFlywheelScale.subscribe(scale -> flywheelEncoder.setConversionFactor(scale), true);

        var shooterGroup = MoShuffleboard.getInstance()
                .matchTab
                .getLayout("Shooter", BuiltInLayouts.kList)
                .withSize(2, 1)
                .withProperties(Map.of("Label position", "LEFT"));
        shooterGroup.addDouble(
                "Roller Pos. (cm)", () -> rollerEncoder.getPosition().in(Units.Centimeters));
        shooterGroup.addDouble(
                "Flywheel Vel. (rps)", () -> flywheelEncoder.getVelocity().in(Units.RotationsPerSecond));

        rollerPosPid = new MoSparkMaxPID<Distance>(Type.SMARTMOTION, roller, 0, rollerEncoder);
        TunerUtils.forMoSparkMax(rollerPosPid, "Shooter Roller Pos.");

        flywheelVelocityPid = new MoSparkMaxPID<Angle>(Type.VELOCITY, flywheelLeft, 0, flywheelEncoder);
        TunerUtils.forMoSparkMax(flywheelVelocityPid, "Shooter Flywheel Vel.");
    }

    public void directDriveRoller(double speed) {
        roller.set(speed);
    }

    public void setRollerPos(Measure<Distance> pos) {
        rollerPosPid.setPositionReference(pos);
    }

    public void directDriveFlywheel(double speed) {
        flywheelLeft.set(speed);
    }

    public void setFlywheelSpeed(Measure<Velocity<Angle>> speed) {
        flywheelVelocityPid.setVelocityReference(speed);
    }
}
