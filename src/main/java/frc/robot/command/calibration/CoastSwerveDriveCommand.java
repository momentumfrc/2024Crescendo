package frc.robot.command.calibration;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.DriveSubsystem;

public class CoastSwerveDriveCommand extends Command {
    private DriveSubsystem drive;

    public CoastSwerveDriveCommand(DriveSubsystem drive) {
        this.drive = drive;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.forEachSwerveModule((module) -> {
            module.driveMotor.setNeutralMode(NeutralModeValue.Coast);
        });
    }

    @Override
    public void execute() {
        drive.forEachSwerveModule((module) -> {
            module.driveMotor.setControl(new DutyCycleOut(0));
            module.turnPID.setReference(0);
        });
    }

    @Override
    public void end(boolean interrupted) {
        drive.forEachSwerveModule((module) -> {
            module.driveMotor.setNeutralMode(NeutralModeValue.Brake);
        });
    }
}
