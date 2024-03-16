package frc.robot.command.shooter;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.util.MoPrefs;

public class ShootSpeakerCommand extends Command {
    private final ShooterSubsystem shooter;

    private MutableMeasure<Distance> startRollerPos = MutableMeasure.zero(Units.Centimeters);

    private boolean upToSpeed = false;

    private Timer timer = new Timer();

    public ShootSpeakerCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;

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
        var flywheelSpeed = MoPrefs.flywheelSpeakerSetpoint.get();
        shooter.setFlywheelSpeed(flywheelSpeed);

        double thresh = MoPrefs.shooterSetpointVarianceThreshold.get().in(Units.Value);
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
        var rollerFeedTime = MoPrefs.shooterRollerRunTime.get();
        return upToSpeed && timer.hasElapsed(rollerFeedTime.in(Units.Seconds));
    }
}
