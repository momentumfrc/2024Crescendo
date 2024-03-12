package frc.robot.command.shooter;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.util.MoUnits;

public class IdleShooterCommand extends Command {
    private static final Measure<Velocity<Distance>> IDLE_SPEED = MoUnits.CentimetersPerSec.zero();

    private final ShooterSubsystem shooter;

    private MutableMeasure<Distance> rollerPos = MutableMeasure.zero(Units.Centimeter);

    public IdleShooterCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        rollerPos.mut_replace(shooter.getRollerPosition());
    }

    @Override
    public void execute() {
        shooter.setRollerPosition(rollerPos);
        shooter.setFlywheelSpeed(IDLE_SPEED);
    }
}
