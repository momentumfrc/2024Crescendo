package frc.robot.command.shooter;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.util.MoPrefs;

public class ShootShooterCommand extends Command {
    private final ShooterSubsystem shooter;
    private final Measure<Distance> rollerFeed;
    private final Measure<Velocity<Angle>> flywheelSpeed;

    private MutableMeasure<Distance> startRollerPos = MutableMeasure.zero(Units.Centimeters);
    private MutableMeasure<Distance> targetRollerPos = MutableMeasure.zero(Units.Centimeters);
    private boolean upToSpeed = false;

    public ShootShooterCommand(
            ShooterSubsystem shooter, Measure<Distance> rollerFeed, Measure<Velocity<Angle>> flywheelSpeed) {
        this.shooter = shooter;
        this.rollerFeed = rollerFeed;
        this.flywheelSpeed = flywheelSpeed;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        upToSpeed = false;
        var pos = shooter.rollerEncoder.getPosition();
        startRollerPos.mut_replace(pos);
        targetRollerPos.mut_replace(pos).mut_plus(rollerFeed);
    }

    @Override
    public void execute() {
        shooter.setFlywheelSpeed(flywheelSpeed);

        double thresh = MoPrefs.pidSetpointVarianceThreshold.get().in(Units.Value);
        if (shooter.flywheelEncoder.getVelocity().isNear(flywheelSpeed, thresh)) {
            upToSpeed = true;
        }

        if (upToSpeed) {
            shooter.setRollerPos(targetRollerPos);
        } else {
            shooter.setRollerPos(startRollerPos);
        }
    }

    @Override
    public boolean isFinished() {
        double thresh = MoPrefs.pidSetpointVarianceThreshold.get().in(Units.Value);
        return shooter.rollerEncoder.getPosition().isNear(targetRollerPos, thresh);
    }
}
