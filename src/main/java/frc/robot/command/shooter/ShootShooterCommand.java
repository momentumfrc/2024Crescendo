package frc.robot.command.shooter;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.util.MoPrefs;

public class ShootShooterCommand extends Command {
    private final ShooterSubsystem shooter;
    private final Measure<Time> rollerFeedTime;
    private final Measure<Velocity<Distance>> flywheelSpeed;

    private MutableMeasure<Distance> startRollerPos = MutableMeasure.zero(Units.Centimeters);

    private boolean upToSpeed = false;

    private Timer timer = new Timer();

    public ShootShooterCommand(
            ShooterSubsystem shooter, Measure<Time> rollerFeedTime, Measure<Velocity<Distance>> flywheelSpeed) {
        this.shooter = shooter;
        this.rollerFeedTime = rollerFeedTime;
        this.flywheelSpeed = flywheelSpeed;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        upToSpeed = false;
        startRollerPos.mut_replace(shooter.getRollerPosition());

        timer.restart();
    }

    @Override
    public void execute() {
        shooter.setFlywheelSpeed(flywheelSpeed);

        double thresh = MoPrefs.pidSetpointVarianceThreshold.get().in(Units.Value);
        if (shooter.getAvgFlywheelVelocity().isNear(flywheelSpeed, thresh)) {
            upToSpeed = true;
        }

        if (upToSpeed) {
            shooter.directDriveRoller(1);
        } else {
            shooter.setRollerPosition(startRollerPos);
            timer.restart();
        }
    }

    @Override
    public boolean isFinished() {
        return upToSpeed && timer.hasElapsed(rollerFeedTime.in(Units.Seconds));
    }
}
