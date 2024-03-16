package frc.robot.command.shooter;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.util.MoPrefs;

public class ShootAmpCommand extends Command {
    private final ShooterSubsystem shooter;

    private MutableMeasure<Velocity<Distance>> flywheelSpeed = MutableMeasure.zero(Units.MetersPerSecond);

    public ShootAmpCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        flywheelSpeed.mut_replace(MoPrefs.shooterFlywheelReverseSpeed.get());
        flywheelSpeed.mut_times(-1);
        shooter.setFlywheelSpeed(flywheelSpeed);

        shooter.directDriveRoller(-MoPrefs.shooterRollerReversePower.get().in(Units.Value));
    }
}
