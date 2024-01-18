// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.input.MoInput;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.PositioningSubsystem;
import frc.robot.util.MoPrefs;
import frc.robot.util.MoPrefs.Pref;
import java.util.function.Supplier;

public class TeleopDriveCommand extends Command {
    private final DriveSubsystem drive;
    private final PositioningSubsystem positioning;
    private final Supplier<MoInput> inputSupplier;

    private Pref<Double> rampTime = MoPrefs.driveRampTime;

    private SlewRateLimiter fwdLimiter;
    private SlewRateLimiter leftLimiter;
    private SlewRateLimiter turnLimiter;

    public TeleopDriveCommand(
            DriveSubsystem drive,
            PositioningSubsystem positioning,
            Supplier<MoInput> inputSupplier) {
        this.drive = drive;
        this.positioning = positioning;
        this.inputSupplier = inputSupplier;

        rampTime.subscribe(
                rampTime -> {
                    double slewRate = 1.0 / rampTime;

                    fwdLimiter = new SlewRateLimiter(slewRate);
                    leftLimiter = new SlewRateLimiter(slewRate);
                    turnLimiter = new SlewRateLimiter(slewRate);
                },
                true);

        addRequirements(drive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        MoInput input = inputSupplier.get();
        var mvRequest = input.getMoveRequest();
        double fwdRequest = mvRequest.y();
        double leftRequest = mvRequest.x();

        double turnRequest = input.getTurnRequest();

        if (input.getShouldUseSlowSpeed()) {
            double slowSpeed = MoPrefs.driveSlowSpeed.get();
            double turnSlowSpeed = MoPrefs.turnSlowSpeed.get();
            fwdRequest *= slowSpeed;
            leftRequest *= slowSpeed;
            turnRequest *= turnSlowSpeed;
        }

        fwdRequest = fwdLimiter.calculate(fwdRequest);
        leftRequest = leftLimiter.calculate(leftRequest);
        turnRequest = turnLimiter.calculate(turnRequest);

        var foHeading = positioning.getFieldOrientedDriveHeading();
        drive.driveCartesian(fwdRequest, leftRequest, turnRequest, foHeading);

        if (input.getReZeroGyro()) {
            this.positioning.resetFieldOrientedFwd();
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
