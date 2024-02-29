// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.calibration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.input.MoInput;
import frc.robot.subsystem.DriveSubsystem;
import java.util.function.Supplier;

public class CalibrateSwerveTurnCommand extends Command {

    private final DriveSubsystem drive;
    private final Supplier<MoInput> inputSupplier;

    public CalibrateSwerveTurnCommand(DriveSubsystem drive, Supplier<MoInput> inputSupplier) {
        this.drive = drive;
        this.inputSupplier = inputSupplier;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        DriverStation.reportWarning("ENTERING DRIVE TURN CALIBRATION", false);

        drive.doResetEncoders = false;
    }

    @Override
    public void execute() {
        var mvRequest = inputSupplier.get().getMoveRequest();
        double fwdRequest = mvRequest.y();
        double leftRequest = mvRequest.x();

        SwerveModuleState state = new SwerveModuleState(0, new Rotation2d(-leftRequest, -fwdRequest));

        drive.frontLeft.drive(state);
        drive.frontRight.drive(state);
        drive.rearLeft.drive(state);
        drive.rearRight.drive(state);
    }

    @Override
    public void end(boolean wasInterrupted) {
        DriverStation.reportWarning("FINISHING DRIVE TURN CALIBRATION", false);

        drive.doResetEncoders = true;
    }
}
