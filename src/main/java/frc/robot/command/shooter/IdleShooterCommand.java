package frc.robot.command.shooter;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.ShooterSubsystem;

public class IdleShooterCommand extends Command {
    private static final Measure<Velocity<Angle>> IDLE_SPEED = Units.RotationsPerSecond.of(0);

    private final ShooterSubsystem shooter;

    private MutableMeasure<Distance> rollerPos = MutableMeasure.zero(Units.Centimeter);

    public IdleShooterCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        rollerPos.mut_replace(shooter.rollerEncoder.getPosition());
    }

    @Override
    public void execute() {
        shooter.setRollerPos(rollerPos);
        shooter.setFlywheelSpeed(IDLE_SPEED);
    }
}
