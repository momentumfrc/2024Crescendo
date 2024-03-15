// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.climb;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.component.Climber;
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
        climb.leftClimber.invalidateZero();

        climb.rightClimber.enableWinchSoftLimitReverse(false);
        climb.rightClimber.invalidateZero();
    }

    private static void zeroSide(double power, Climber climber, Timer currentTimer) {
        if (climber.hasZero()) {
            climber.runWinch(0);

            return;
        }

        if (climber.winch.getOutputCurrent()
                >= MoPrefs.climberZeroCurrentCutoff.get().in(Units.Amps)) {
            if (currentTimer.hasElapsed(MoPrefs.climberZeroTimeCutoff.get().in(Units.Seconds))) {
                climber.setZero(MoPrefs.climberZeroPosition.get());
            }
        } else {
            currentTimer.restart();
        }

        climber.runWinch(-Math.abs(MoPrefs.intakeZeroPwr.get()));
    }

    @Override
    public void execute() {
        zeroSide(MoPrefs.climberZeroPwr.get(), climb.leftClimber, leftCurrentTimer);
        zeroSide(MoPrefs.climberZeroPwr.get(), climb.rightClimber, rightCurrentTimer);
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
