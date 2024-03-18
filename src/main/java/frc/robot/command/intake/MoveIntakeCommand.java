// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.intake;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.component.IntakeSetpointManager;
import frc.robot.component.IntakeSetpointManager.IntakeSetpoint;
import frc.robot.subsystem.IntakeSubsystem;

public class MoveIntakeCommand extends Command {
    private final IntakeSubsystem intake;
    private final Measure<Angle> position;

    public MoveIntakeCommand(IntakeSubsystem intake, Measure<Angle> toPosition) {
        this.intake = intake;
        this.position = toPosition;

        addRequirements(intake);
    }

    public static Command forSetpoint(IntakeSubsystem intake, IntakeSetpoint setpoint) {
        return new MoveIntakeCommand(intake, IntakeSetpointManager.getInstance().getSetpoint(setpoint))
                .alongWith(Commands.runOnce(() -> intake.setpointPublisher.setString(setpoint.toString()), intake));
    }

    @Override
    public void execute() {
        intake.deploySmartMotion(position);
    }
}
