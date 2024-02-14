package frc.robot.command;

import com.ctre.phoenix6.Orchestra;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.input.MoInput;
import frc.robot.subsystem.DriveSubsystem;
import java.util.function.Supplier;

public class OrchestraCommand extends Command {
    public static final double CANCEL_CUTOFF = 0.05;

    private final DriveSubsystem drive;
    private final Supplier<MoInput> inputSupplier;

    private final Orchestra orchestra;

    public OrchestraCommand(DriveSubsystem drive, Supplier<MoInput> inputSupplier, String chrpFile) {
        this.drive = drive;
        this.inputSupplier = inputSupplier;

        this.orchestra = new Orchestra();
        orchestra.addInstrument(drive.rearLeft.driveMotor, 0);
        orchestra.addInstrument(drive.rearRight.driveMotor, 0);
        orchestra.addInstrument(drive.frontLeft.driveMotor, 0);
        orchestra.addInstrument(drive.frontRight.driveMotor, 0);

        orchestra.loadMusic(chrpFile);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        orchestra.play();
    }

    @Override
    public void execute() {
        drive.rearLeft.turnMotor.set(0);
        drive.rearRight.turnMotor.set(0);
        drive.frontLeft.turnMotor.set(0);
        drive.frontRight.turnMotor.set(0);
    }

    @Override
    public boolean isFinished() {
        var input = inputSupplier.get();
        boolean hasDriveRequest =
                input.getMoveRequest().len() > CANCEL_CUTOFF || Math.abs(input.getTurnRequest()) > CANCEL_CUTOFF;
        return !orchestra.isPlaying() || hasDriveRequest;
    }

    @Override
    public void end(boolean interrupted) {
        orchestra.stop();
    }
}
