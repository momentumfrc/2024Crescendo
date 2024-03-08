package frc.robot.command.intake;

import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.IntakeSubsystem;
import frc.robot.util.MoPrefs;

public class ZeroIntakeCommand extends Command {
    private IntakeSubsystem intake;
    private Timer currentTimer;

    private MutableMeasure<Distance> initialRollerPos;

    public ZeroIntakeCommand(IntakeSubsystem intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        currentTimer.stop();
        currentTimer.reset();

        initialRollerPos.mut_replace(intake.getRollerPosition());
    }

    @Override
    public void execute() {
        Measure<Current> current = intake.getDeployCurrent();

        if (current.gte(MoPrefs.intakeZeroCurrentCutoff.get())) {
            if (currentTimer.hasElapsed(MoPrefs.intakeZeroTimeCutoff.get().in(Units.Seconds))) {
                intake.zeroDeployEncoder(MoPrefs.intakeZeroPosition.get());
                intake.isDeployZeroed.setBoolean(true);
            }
        } else {
            currentTimer.restart();
        }

        switch (intake.controlMode.getSelected()) {
            case SMARTMOTION:
                intake.intakeSmartMotion(initialRollerPos);
                break;
            case DIRECT_VELOCITY:
                intake.intakeVelocity(Units.MetersPerSecond.zero());
                break;
            case FALLBACK_DIRECT_POWER:
                intake.intakeFallbackDirectPower(0);
                break;
        }

        intake.deployFallbackDirectPower(MoPrefs.intakeZeroPwr.get());
    }

    @Override
    public boolean isFinished() {
        return intake.isDeployZeroed.getBoolean(false);
    }
}
