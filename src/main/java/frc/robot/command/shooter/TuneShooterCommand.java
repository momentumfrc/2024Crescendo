package frc.robot.command.shooter;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.util.MoPrefs;
import frc.robot.util.MoUnits;

public class TuneShooterCommand extends Command {
    private final ShooterSubsystem shooter;

    public TuneShooterCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        var flywheelSetpoint = MoPrefs.flywheelSpeakerSetpoint.get();
        shooter.setFlywheelSpeed(flywheelSetpoint);

        double thresh = MoPrefs.pidSetpointVarianceThreshold.get().in(Units.Value);

        if (shooter.getAvgFlywheelVelocity().isNear(flywheelSetpoint, thresh)) {
            shooter.setRollerVelocity(flywheelSetpoint);
        } else {
            shooter.setRollerVelocity(MoUnits.CentimetersPerSec.zero());
        }
    }
}
