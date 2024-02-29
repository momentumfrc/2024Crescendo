package frc.robot.command;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.input.MoInput;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.PositioningSubsystem;
import frc.robot.util.MoPrefs;
import frc.robot.util.MoPrefs.Pref;
import java.util.function.Supplier;

public class TeleopDriveWithPointAtCommand extends Command {
    private DriveSubsystem drive;
    private final PositioningSubsystem positioning;
    private final Supplier<MoInput> inputSupplier;

    private Pref<Double> rampTime = MoPrefs.driveRampTime;

    private SlewRateLimiter fwdLimiter;
    private SlewRateLimiter leftLimiter;

    private Pose2d pointAtTarget;

    public TeleopDriveWithPointAtCommand(
            DriveSubsystem drive,
            PositioningSubsystem positioning,
            Supplier<MoInput> inputSupplier,
            Pose2d pointAtTarget) {
        this.drive = drive;

        this.positioning = positioning;
        this.inputSupplier = inputSupplier;

        this.pointAtTarget = pointAtTarget;

        rampTime.subscribe(
                rampTime -> {
                    double slewRate = 1.0 / rampTime;

                    fwdLimiter = new SlewRateLimiter(slewRate);
                    leftLimiter = new SlewRateLimiter(slewRate);
                },
                true);

        addRequirements(drive);
    }

    @Override
    public void execute() {
        MoInput input = inputSupplier.get();
        var mvRequest = input.getMoveRequest();
        double fwdRequest = mvRequest.y();
        double leftRequest = mvRequest.x();

        if (input.getShouldUseSlowSpeed()) {
            double slowSpeed = MoPrefs.driveSlowSpeed.get();
            fwdRequest *= slowSpeed;
            leftRequest *= slowSpeed;
        }

        fwdRequest = fwdLimiter.calculate(fwdRequest);
        leftRequest = leftLimiter.calculate(leftRequest);

        Pose2d currentRobotPosition = positioning.getAbsoluteRobotPose();
        Transform2d currentRobotOffset = pointAtTarget.minus(currentRobotPosition);
        Rotation2d rotationOffset =
                currentRobotOffset.getTranslation().getAngle().plus(Rotation2d.fromRotations(0.5));

        var foHeading = positioning.getFieldOrientedDriveHeading();
        drive.driveCartesianPointAt(fwdRequest, leftRequest, foHeading, rotationOffset);

        if (input.getReZeroGyro()) {
            this.positioning.resetFieldOrientedFwd();
        }
    }
}
