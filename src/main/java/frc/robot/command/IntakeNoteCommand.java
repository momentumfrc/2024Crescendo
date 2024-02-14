package frc.robot.command;

import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.IntakeSubsystem;

public class IntakeNoteCommand extends Command {
    private static final double INTAKE_SPEED = 0.2;

    private static final Measure<Current> CURRENT_THRESHOLD = Units.Amps.of(4);
    private static final Measure<Time> CURRENT_TIMEOUT = Units.Seconds.of(1);

    private IntakeSubsystem intake;
    private Timer timer = new Timer();

    public IntakeNoteCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.set(INTAKE_SPEED);
    }

    @Override
    public boolean isFinished() {
        if (intake.getCurrent().gte(CURRENT_THRESHOLD)) {
            return timer.hasElapsed(CURRENT_TIMEOUT.in(Units.Seconds));
        } else {
            timer.restart();
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.set(0);
    }
}
