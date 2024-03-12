package frc.robot.command.shooter;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.util.MoPrefs;

public class SpinupShooterCommand extends Command {
    private final ShooterSubsystem shooter;

    private final Measure<Velocity<Distance>> flywheelSpeed;
    private final MutableMeasure<Distance> rollerPos = MutableMeasure.zero(Units.Centimeters);

    public SpinupShooterCommand(ShooterSubsystem shooter, Measure<Velocity<Distance>> flywheelSpeed) {
        this.shooter = shooter;

        this.flywheelSpeed = flywheelSpeed;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        rollerPos.mut_replace(shooter.getRollerPosition());
    }

    @Override
    public void execute() {
        shooter.setRollerPos(rollerPos);
        shooter.setFlywheelSpeed(flywheelSpeed);
    }

    public boolean onTarget() {
        double thresh = MoPrefs.pidSetpointVarianceThreshold.get().in(Units.Value);
        return shooter.getAvgFlywheelVelocity().isNear(flywheelSpeed, thresh);
    }
}
