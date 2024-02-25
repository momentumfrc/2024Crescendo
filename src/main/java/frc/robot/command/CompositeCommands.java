package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.command.shooter.ShootShooterCommand;
import frc.robot.command.shooter.SpinupShooterCommand;
import frc.robot.subsystem.ArmSubsystem;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.PositioningSubsystem;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.util.MoPrefs;

public class CompositeCommands {

    public static Command shootSpeakerCommand(
            ArmSubsystem arm, DriveSubsystem drive, ShooterSubsystem shooter, PositioningSubsystem pos) {
        var flywheelSpeed = MoPrefs.shooterFlywheelSetpoint.get();
        var rollerSetpoint = MoPrefs.shooterRollerSetpoint.get();

        var aimSpeakerCommand = new AimSpeakerCommand(arm, drive, pos);
        var spinupShooterCommand = new SpinupShooterCommand(shooter, flywheelSpeed);
        var shootShooterCommand = new ShootShooterCommand(shooter, rollerSetpoint, flywheelSpeed);
        var waitForAim = Commands.waitUntil(aimSpeakerCommand::onTarget);
        var waitForSpinup = Commands.waitUntil(spinupShooterCommand::onTarget);

        return ((waitForAim.andThen(waitForSpinup))
                        .deadlineWith(spinupShooterCommand)
                        .andThen(shootShooterCommand))
                .deadlineWith(aimSpeakerCommand);
    }

    private CompositeCommands() {
        throw new UnsupportedOperationException("CompositeCommands is a static class and should not be instantiated");
    }
}
