package frc.robot.command.intake;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.component.IntakeSetpointManager;
import frc.robot.component.IntakeSetpointManager.IntakeSetpoint;
import frc.robot.input.MoInput;
import frc.robot.subsystem.IntakeSubsystem;
import frc.robot.util.MoPrefs;
import frc.robot.util.PositionOverrideUtil;
import java.util.function.Supplier;

public class RunIntakeCommand extends Command {
    private static final double INTAKE_OVERRIDE_TIMEOUT = 3;

    private enum State {
        DEPLOY,
        INTAKE,
        RETRACT
    };

    private final IntakeSubsystem intake;
    private final Supplier<MoInput> inputSupplier;

    private final PositionOverrideUtil positionOverride;
    private final MutableMeasure<Distance> holdNotePosition = MutableMeasure.zero(Units.Centimeters);

    private final Timer currentSenseTimer = new Timer();

    private State state = State.DEPLOY;

    public RunIntakeCommand(IntakeSubsystem intake, Supplier<MoInput> inputSupplier) {
        this.intake = intake;
        this.inputSupplier = inputSupplier;

        this.positionOverride = new PositionOverrideUtil(
                intake::getDeployPosition,
                intake::deploySmartMotion,
                intake::deployVelocity,
                MoPrefs.intakeDeployMaxSpeed,
                INTAKE_OVERRIDE_TIMEOUT);

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        currentSenseTimer.reset();
        positionOverride.reset();
        state = State.DEPLOY;

        if (intake.getIsHoldingNote()) {
            state = State.RETRACT;
            holdNotePosition.mut_replace(intake.getRollerPosition());
        }
    }

    private void executeSmartMotion() {
        double tolerance = MoPrefs.intakeSetpointVarianceThreshold.get().in(Units.Value);
        var setpointManager = IntakeSetpointManager.getInstance();
        var intakeSetpoint = setpointManager.getSetpoint(IntakeSetpoint.INTAKE);
        var handoffSetpoint = setpointManager.getSetpoint(IntakeSetpoint.HANDOFF);

        var moInput = inputSupplier.get();
        double deployAdjust = moInput.getIntakeAdjust();
        // Transition state
        switch (state) {
            case DEPLOY:
                if (intake.getDeployPosition().isNear(intakeSetpoint, tolerance)) {
                    state = State.INTAKE;
                }
                break;
            case INTAKE:
                if (intake.getRollerCurrent().gte(MoPrefs.intakeCurrentSenseThreshold.get())) {
                    if (currentSenseTimer.hasElapsed(
                            MoPrefs.intakeCurrentSenseTime.get().in(Units.Seconds))) {
                        state = State.RETRACT;
                        holdNotePosition.mut_replace(intake.getRollerPosition());
                    }
                } else {
                    currentSenseTimer.restart();
                }
                break;
            case RETRACT:
                break;
        }

        // Execute logic for current state
        switch (state) {
            case DEPLOY:
                positionOverride.runSmartMotionWithAdjust(intakeSetpoint, deployAdjust);
                intake.intakeVelocity(Units.MetersPerSecond.zero());
                break;
            case INTAKE:
                positionOverride.runSmartMotionWithAdjust(intakeSetpoint, deployAdjust);
                intake.intakeVelocity(MoPrefs.intakeRollerSpeed.get());
                break;
            case RETRACT:
                positionOverride.runSmartMotionWithAdjust(handoffSetpoint, deployAdjust);
                intake.intakeSmartMotion(holdNotePosition);
                break;
        }
    }

    private void executeDirectVelocity() {
        var moInput = inputSupplier.get();
        double deployAdjust = moInput.getIntakeAdjust();

        switch (state) {
            case DEPLOY:
                state = State.INTAKE;
                break;
            case INTAKE:
                if (intake.getRollerCurrent().gte(MoPrefs.intakeCurrentSenseThreshold.get())) {
                    if (currentSenseTimer.hasElapsed(
                            MoPrefs.intakeCurrentSenseTime.get().in(Units.Seconds))) {
                        state = State.RETRACT;
                    }
                } else {
                    currentSenseTimer.restart();
                }
                break;
            case RETRACT:
                break;
        }

        // Execute logic for current state
        switch (state) {
            case DEPLOY:
                positionOverride.runVelocity(deployAdjust);
                intake.intakeVelocity(Units.MetersPerSecond.zero());
                break;
            case INTAKE:
                positionOverride.runVelocity(deployAdjust);
                intake.intakeVelocity(MoPrefs.intakeRollerSpeed.get());
                break;
            case RETRACT:
                positionOverride.runVelocity(deployAdjust);
                intake.intakeVelocity(Units.MetersPerSecond.zero());
                break;
        }
    }

    private void executeFallbackDirectPower() {
        var moInput = inputSupplier.get();
        double deployAdjust = moInput.getIntakeAdjust();

        switch (state) {
            case DEPLOY:
                state = State.INTAKE;
                break;
            case INTAKE:
                if (intake.getRollerCurrent().gte(MoPrefs.intakeCurrentSenseThreshold.get())) {
                    if (currentSenseTimer.hasElapsed(
                            MoPrefs.intakeCurrentSenseTime.get().in(Units.Seconds))) {
                        state = State.RETRACT;
                    }
                } else {
                    currentSenseTimer.restart();
                }
                break;
            case RETRACT:
                break;
        }

        // Execute logic for current state
        switch (state) {
            case DEPLOY:
                intake.deployFallbackDirectPower(deployAdjust);
                intake.intakeVelocity(Units.MetersPerSecond.zero());
                break;
            case INTAKE:
                intake.deployFallbackDirectPower(deployAdjust);
                intake.intakeVelocity(MoPrefs.intakeRollerSpeed.get());
                break;
            case RETRACT:
                intake.deployFallbackDirectPower(deployAdjust);
                intake.intakeFallbackDirectPower(0);
                break;
        }
    }

    @Override
    public void execute() {
        var controlMode = intake.controlMode.getSelected();

        switch (controlMode) {
            case SMARTMOTION:
                executeSmartMotion();
                return;
            case DIRECT_VELOCITY:
                executeDirectVelocity();
                return;
            case FALLBACK_DIRECT_POWER:
                executeFallbackDirectPower();
                return;
        }
    }

    @Override
    public void end(boolean interrupted) {
        currentSenseTimer.stop();
    }
}
