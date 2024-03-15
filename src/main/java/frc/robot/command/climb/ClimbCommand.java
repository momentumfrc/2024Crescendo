// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.input.MoInput;
import frc.robot.subsystem.ClimbSubsystem;
import java.util.function.Supplier;

public class ClimbCommand extends Command {
    private ClimbSubsystem climb;
    private Supplier<MoInput> inputSupplier;

    public ClimbCommand(ClimbSubsystem climb, Supplier<MoInput> inputSupplier) {
        this.climb = climb;
        this.inputSupplier = inputSupplier;

        addRequirements(climb);
    }

    @Override
    public void execute() {
        MoInput input = inputSupplier.get();

        double leftClimbRequest = input.getLeftClimbRequest();
        double rightClimbRequest = input.getRightClimbRequest();

        climb.leftClimber.runWinch(leftClimbRequest);
        climb.rightClimber.runWinch(rightClimbRequest);
    }

    @Override
    public void end(boolean interrupted) {
        this.climb.stop();
    }
}
