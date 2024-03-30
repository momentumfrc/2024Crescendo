package frc.robot.command.shooter;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.util.MoPrefs;
import java.util.function.Supplier;

public class SpinupShooterCommand extends Command {
    private final ShooterSubsystem shooter;

    private final Supplier<Measure<Velocity<Distance>>> flywheelSpeedSupplier;
    private final MutableMeasure<Distance> rollerPos = MutableMeasure.zero(Units.Centimeters);

    public SpinupShooterCommand(ShooterSubsystem shooter, Supplier<Measure<Velocity<Distance>>> flywheelSpeedSupplier) {
        this.shooter = shooter;

        this.flywheelSpeedSupplier = flywheelSpeedSupplier;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        rollerPos.mut_replace(shooter.getRollerPosition());
    }

    @Override
    public void execute() {
        shooter.setRollerPosition(rollerPos);
        shooter.setFlywheelSpeed(flywheelSpeedSupplier.get());
    }

    public boolean onTarget() {
        var flywheelSpeed = flywheelSpeedSupplier.get();

        return shooter.getAvgFlywheelVelocity().gte(flywheelSpeed.minus(MoPrefs.shooterSetpointThreshold.get()));
    }
}
