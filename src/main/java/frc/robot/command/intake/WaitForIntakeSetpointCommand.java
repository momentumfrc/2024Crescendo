// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.intake;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.component.IntakeSetpointManager;
import frc.robot.component.IntakeSetpointManager.IntakeSetpoint;
import frc.robot.subsystem.IntakeSubsystem;
import frc.robot.util.MoPrefs;

public class WaitForIntakeSetpointCommand extends Command {
    private final IntakeSubsystem intake;
    private final IntakeSetpoint waitForSetpoint;

    public WaitForIntakeSetpointCommand(IntakeSubsystem intake, IntakeSetpoint waitForSetpoint) {
        this.intake = intake;
        this.waitForSetpoint = waitForSetpoint;
    }

    @Override
    public boolean isFinished() {
        Measure<Angle> requestedPos = IntakeSetpointManager.getInstance().getSetpoint(waitForSetpoint);
        double tolerance = MoPrefs.intakeSetpointVarianceThreshold.get().in(Units.Value);

        return intake.getDeployPosition().isNear(requestedPos, tolerance);
    }
}
