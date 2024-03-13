// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.ClimbSubsystem;

public class ZeroClimbersCommand extends Command {
    private static final double POWER = 0.3;

    private final ClimbSubsystem climb;

    public ZeroClimbersCommand(ClimbSubsystem climb) {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void execute() {
        climb.leftClimber.zero(POWER);
        climb.rightClimber.zero(POWER);
    }

    @Override
    public boolean isFinished() {
        return climb.leftClimber.hasZero() && climb.rightClimber.hasZero();
    }
}
