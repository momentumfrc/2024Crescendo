// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.climb;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.ClimbSubsystem;
import frc.robot.util.MoPrefs;

public class ZeroClimbersCommand extends Command {
    private final ClimbSubsystem climb;
    private Timer leftCurrentTimer = new Timer();
    private Timer rightCurrentTimer = new Timer();

    public ZeroClimbersCommand(ClimbSubsystem climb) {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        leftCurrentTimer.stop();
        leftCurrentTimer.reset();

        rightCurrentTimer.stop();
        rightCurrentTimer.reset();

        climb.leftClimber.enableWinchSoftLimitReverse(false);
        climb.rightClimber.enableWinchSoftLimitReverse(false);
    }

    @Override
    public void execute() {
        climb.leftClimber.zero(MoPrefs.climberZeroPwr.get(), leftCurrentTimer);
        climb.rightClimber.zero(MoPrefs.climberZeroPwr.get(), rightCurrentTimer);
    }

    @Override
    public boolean isFinished() {
        return climb.leftClimber.hasZero() && climb.rightClimber.hasZero();
    }

    @Override
    public void end(boolean interrupted) {
        climb.leftClimber.enableWinchSoftLimitReverse(true);
        climb.rightClimber.enableWinchSoftLimitReverse(true);
    }
}
