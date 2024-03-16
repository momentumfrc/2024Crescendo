package frc.robot.command;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.util.MoPrefs;

public class IntakeSourceCommand extends Command {
    private final ShooterSubsystem shooter;

    private final Timer currentSenseTimer = new Timer();

    public IntakeSourceCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        currentSenseTimer.restart();
    }

    @Override
    public void execute() {
        shooter.directDriveRoller(MoPrefs.handoffShooterRollerPower.get().in(Units.Value));
        shooter.setFlywheelSpeed(Units.MetersPerSecond.zero());
    }

    @Override
    public boolean isFinished() {
        if (shooter.getRollerCurrent().gte(MoPrefs.handoffCurrentCutoff.get())) {
            return currentSenseTimer.hasElapsed(MoPrefs.handoffTimeCutoff.get().in(Units.Seconds));
        } else {
            currentSenseTimer.restart();
            return false;
        }
    }
}
