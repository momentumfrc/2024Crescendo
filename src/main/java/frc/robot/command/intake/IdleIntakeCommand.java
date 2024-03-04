package frc.robot.command.intake;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.component.IntakeSetpointManager;
import frc.robot.component.IntakeSetpointManager.IntakeSetpoint;
import frc.robot.input.MoInput;
import frc.robot.subsystem.IntakeSubsystem;
import frc.robot.util.MoPrefs;
import java.util.function.Supplier;

public class IdleIntakeCommand extends Command {
    private static final double INTAKE_OVERRIDE_TIMEOUT = 5;

    private enum State {
        ACTIVE_ADJUST,
        WAIT_FOR_TIMEOUT,
        HOLD_STOW
    };

    private final IntakeSubsystem intake;
    private final Supplier<MoInput> inputSupplier;

    private final Timer timeoutTimer = new Timer();
    private final MutableMeasure<Angle> deployPosition = MutableMeasure.zero(Units.Rotations);
    private final MutableMeasure<Velocity<Angle>> overrideVelocity = MutableMeasure.zero(Units.RotationsPerSecond);
    private final MutableMeasure<Distance> rollerPosition = MutableMeasure.zero(Units.Centimeters);

    private State state = State.HOLD_STOW;

    public IdleIntakeCommand(IntakeSubsystem intake, Supplier<MoInput> inputSupplier) {
        this.intake = intake;
        this.inputSupplier = inputSupplier;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        state = State.HOLD_STOW;
        rollerPosition.mut_replace(intake.getRollerPosition());
    }

    private void holdRollerPosition() {
        var controlMode = intake.controlMode.getSelected();

        switch (controlMode) {
            case SMARTMOTION:
                intake.intakeSmartMotion(rollerPosition);
                break;
            case DIRECT_VELOCITY:
                intake.intakeVelocity(Units.MetersPerSecond.zero());
                break;
            case FALLBACK_DIRECT_POWER:
                intake.intakeFallbackDirectPower(0);
        }
    }

    private void adjustIntake(double amount) {
        var controlMode = intake.controlMode.getSelected();

        switch (controlMode) {
            case SMARTMOTION:
                if (amount == 0) {
                    intake.deploySmartMotion(deployPosition);
                    break;
                }
                deployPosition.mut_replace(intake.getDeployPosition());
                // FALL THROUGH!
            case DIRECT_VELOCITY:
                overrideVelocity.mut_replace(MoPrefs.intakeDeployMaxSpeed.get());
                overrideVelocity.mut_times(amount);
                intake.deployVelocity(overrideVelocity);
                break;
            case FALLBACK_DIRECT_POWER:
            default:
                intake.deployFallbackDirectPower(amount);
                break;
        }
    }

    private void stowIntake() {
        var controlMode = intake.controlMode.getSelected();

        switch (controlMode) {
            case SMARTMOTION:
                intake.deploySmartMotion(IntakeSetpointManager.getInstance().getSetpoint(IntakeSetpoint.STOW));
                break;
            case DIRECT_VELOCITY:
            case FALLBACK_DIRECT_POWER:
            default:
                adjustIntake(0);
                break;
        }
    }

    @Override
    public void execute() {
        var input = inputSupplier.get();
        double intakeAdjust = input.getIntakeAdjust();

        switch (state) {
            case ACTIVE_ADJUST:
                if (intakeAdjust == 0) {
                    state = State.WAIT_FOR_TIMEOUT;
                }
                break;
            case WAIT_FOR_TIMEOUT:
                if (intakeAdjust != 0) {
                    state = State.ACTIVE_ADJUST;
                } else if (timeoutTimer.hasElapsed(INTAKE_OVERRIDE_TIMEOUT)) {
                    state = State.HOLD_STOW;
                }
                break;
            case HOLD_STOW:
                if (intakeAdjust != 0) {
                    state = State.ACTIVE_ADJUST;
                }
                break;
        }

        switch (state) {
            case ACTIVE_ADJUST:
                adjustIntake(intakeAdjust);
                timeoutTimer.restart();
                break;
            case WAIT_FOR_TIMEOUT:
                adjustIntake(0);
                break;
            case HOLD_STOW:
                stowIntake();
                break;
        }

        if (input.getSaveIntakeSetpoint() && (state != State.HOLD_STOW)) {
            IntakeSetpointManager.getInstance().setSetpoint(IntakeSetpoint.STOW, intake.getDeployPosition());
            state = State.HOLD_STOW;
        }

        holdRollerPosition();
    }
}
