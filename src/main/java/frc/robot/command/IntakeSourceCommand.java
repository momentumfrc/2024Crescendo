package frc.robot.command;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.util.MoPrefs;

public class IntakeSourceCommand extends Command {
    private final ShooterSubsystem shooter;

    public IntakeSourceCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.directDriveRoller(MoPrefs.handoffShooterRollerPower.get().in(Units.Value));
        shooter.setFlywheelSpeed(Units.MetersPerSecond.zero());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(shooter.getAvgFlywheelVelocity().in(Units.MetersPerSecond)) > MoPrefs.backoffZeroTolerance.get();
    }
}
