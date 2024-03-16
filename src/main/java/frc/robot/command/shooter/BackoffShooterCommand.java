package frc.robot.command.shooter;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.util.MoPrefs;

public class BackoffShooterCommand extends Command {
    private ShooterSubsystem shooter;

    private Timer timeoutTimer = new Timer();
    private Timer startTimeTimer = new Timer();

    public BackoffShooterCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        timeoutTimer.restart();
        startTimeTimer.restart();
    }

    @Override
    public void execute() {
        shooter.directDriveFlywheel(0);
        shooter.directDriveRoller(-Math.abs(MoPrefs.backoffPower.get()));
    }

    @Override
    public boolean isFinished() {
        return timeoutTimer.hasElapsed(MoPrefs.backoffTimeout.get())
                || (Math.abs(shooter.getAvgFlywheelVelocity().in(Units.MetersPerSecond))
                                < MoPrefs.backoffZeroTolerance.get()
                        && startTimeTimer.hasElapsed(MoPrefs.backoffStartTime.get()));
    }
}
