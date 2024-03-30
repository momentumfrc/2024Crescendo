package frc.robot.command.shooter;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.input.MoInput;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.util.MoPrefs;
import frc.robot.util.MoUnits;
import java.util.function.Supplier;

public class IdleShooterCommand extends Command {
    public static final Measure<Velocity<Distance>> IDLE_SPEED = MoUnits.CentimetersPerSec.zero();

    private final ShooterSubsystem shooter;
    private final Supplier<MoInput> inputSupplier;

    private MutableMeasure<Distance> rollerPos = MutableMeasure.zero(Units.Centimeter);
    private MutableMeasure<Velocity<Distance>> flywheelSpeed = MutableMeasure.zero(Units.MetersPerSecond);

    public IdleShooterCommand(ShooterSubsystem shooter, Supplier<MoInput> inputSupplier) {
        this.shooter = shooter;
        this.inputSupplier = inputSupplier;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        rollerPos.mut_replace(shooter.getRollerPosition());
    }

    @Override
    public void execute() {
        if (inputSupplier.get().getReverseShooter()) {
            flywheelSpeed.mut_replace(MoPrefs.shooterFlywheelReverseSpeed.get());
            flywheelSpeed.mut_times(-1);

            shooter.setFlywheelSpeed(flywheelSpeed);
            shooter.directDriveRoller(-MoPrefs.shooterRollerReversePower.get().in(Units.Value));
        } else {
            shooter.setFlywheelSpeed(IDLE_SPEED);
            shooter.setRollerPosition(rollerPos);
        }
    }
}
