// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.intake;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.IntakeSubsystem;
import frc.robot.util.MoPrefs;

public class RunIntakeUntilNoteCommand extends Command {

    private final IntakeSubsystem intake;

    private final Timer currentSenseTimer = new Timer();
    private boolean currentTrip = false;

    public RunIntakeUntilNoteCommand(IntakeSubsystem intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        currentSenseTimer.restart();
        currentTrip = false;
    }

    @Override
    public void execute() {
        if (!currentTrip) {
            if (intake.getRollerCurrent().gte(MoPrefs.intakeCurrentSenseThreshold.get())) {
                if (currentSenseTimer.hasElapsed(
                        MoPrefs.intakeCurrentSenseTime.get().in(Units.Seconds))) {
                    currentTrip = true;
                    intake.setIsHoldingNote(true);
                }
            } else {
                intake.setIsHoldingNote(false);
                currentSenseTimer.restart();
            }
        }

        if (!currentTrip) {
            intake.rollerIntakeDirectPower(MoPrefs.intakeRollerPower.get().in(Units.Value));
        } else {
            intake.rollerIntakeDirectPower(0);
        }
    }

    @Override
    public boolean isFinished() {
        return currentTrip;
    }
}
