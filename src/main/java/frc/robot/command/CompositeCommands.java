package frc.robot.command;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.command.arm.AimSpeakerCommand;
import frc.robot.command.arm.MoveArmCommand;
import frc.robot.command.shooter.ShootShooterCommand;
import frc.robot.command.shooter.SpinupShooterCommand;
import frc.robot.component.ArmSetpointManager.ArmSetpoint;
import frc.robot.input.MoInput;
import frc.robot.subsystem.ArmSubsystem;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.PositioningSubsystem;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.util.MoPrefs;
import java.util.function.Supplier;

public class CompositeCommands {

    public static Command shootSpeakerCommand(
            ArmSubsystem arm,
            DriveSubsystem drive,
            ShooterSubsystem shooter,
            PositioningSubsystem pos,
            Supplier<MoInput> getInput) {
        var flywheelSpeed = MoPrefs.flywheelSpeakerSetpoint.get();
        var rollerRunTime = MoPrefs.shooterRollerRunTime.get();

        Pose2d targetPose = pos.getSpeakerPose();

        var pointAtSpeakerCommand = new TeleopDriveWithPointAtCommand(drive, pos, getInput, targetPose);

        var aimSpeakerCommand = new AimSpeakerCommand(arm, pos);
        var spinupShooterCommand = new SpinupShooterCommand(shooter, flywheelSpeed);
        var shootShooterCommand = new ShootShooterCommand(shooter, rollerRunTime, flywheelSpeed);
        var waitForAim = Commands.waitUntil(aimSpeakerCommand::onTarget);
        var waitForSpinup = Commands.waitUntil(spinupShooterCommand::onTarget);

        return ((waitForAim.andThen(waitForSpinup))
                        .deadlineWith(spinupShooterCommand)
                        .andThen(shootShooterCommand))
                .deadlineWith(aimSpeakerCommand, pointAtSpeakerCommand);
    }

    public static Command shootAmpCommand(ArmSubsystem arm, ShooterSubsystem shooter, PositioningSubsystem pos) {
        var flywheelSpeed = MoPrefs.flywheelSpeakerSetpoint.get();
        var rollerRunTime = MoPrefs.shooterRollerRunTime.get();

        var moveArmCommand = MoveArmCommand.forSetpoint(arm, ArmSetpoint.AMP);
        var spinupShooterCommand = new SpinupShooterCommand(shooter, flywheelSpeed);
        var shootShooterCommand = new ShootShooterCommand(shooter, rollerRunTime, flywheelSpeed);
        var waitForArm = Commands.waitUntil(moveArmCommand::onTarget);
        var waitForSpinup = Commands.waitUntil(spinupShooterCommand::onTarget);

        return ((waitForArm.andThen(waitForSpinup))
                        .deadlineWith(spinupShooterCommand)
                        .andThen(shootShooterCommand))
                .deadlineWith(moveArmCommand);
    }

    private CompositeCommands() {
        throw new UnsupportedOperationException("CompositeCommands is a static class and should not be instantiated");
    }
}
