package frc.robot.command.intake;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.component.IntakeSetpointManager;
import frc.robot.component.IntakeSetpointManager.IntakeSetpoint;
import frc.robot.input.MoInput;
import frc.robot.subsystem.IntakeSubsystem;
import frc.robot.subsystem.IntakeSubsystem.IntakeControlMode;
import frc.robot.util.MoPrefs;
import frc.robot.util.MoShuffleboard;
import java.util.function.Supplier;

public class TeleopIntakeCommand extends Command {
    private static final double INTAKE_OVERRIDE_TIMEOUT = 2;

    private final IntakeSubsystem intake;
    private final Supplier<MoInput> inputSupplier;

    private final Timer smartMotionOverrideTimer = new Timer();
    private final Timer currentSenseTimer = new Timer();

    private final GenericPublisher setpointPublisher;

    private boolean smartMotionOverride = false;
    private boolean currentTrip = false;

    private MutableMeasure<Velocity<Angle>> velocityRequest = MutableMeasure.zero(Units.RotationsPerSecond);

    public TeleopIntakeCommand(IntakeSubsystem intake, Supplier<MoInput> inputSupplier) {
        this.intake = intake;
        this.inputSupplier = inputSupplier;

        setpointPublisher = MoShuffleboard.getInstance()
                .intakeTab
                .add("Setpoint", "UNKNOWN")
                .getEntry();

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        smartMotionOverrideTimer.restart();
        currentSenseTimer.restart();
        smartMotionOverride = false;
        currentTrip = false;
    }

    /**
     * If we're running the intake, and we trip the current sense, we should stop the intake motor even if
     * runIntake is still true. We shouldn't try to start running the intake again until runIntake has transitioned
     * to false, then back to true.
     *
     * The boolean currentTrip ensures this.
     */
    private void runIntakeRollerWithCurrentSense(boolean runIntake) {

        if (runIntake && !currentTrip) {
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
        } else if (!runIntake) {
            currentTrip = false;
            currentSenseTimer.restart();
        }

        if (runIntake && !currentTrip) {
            intake.intakeDirectPower(MoPrefs.intakeRollerPower.get().in(Units.Value));
        } else {
            intake.intakeDirectPower(0);
        }
    }

    private void executeSmartMotion() {
        double tolerance = MoPrefs.intakeSetpointVarianceThreshold.get().in(Units.Value);

        var moInput = inputSupplier.get();
        double deployAdjust = moInput.getIntakeAdjust();
        boolean runIntakeRequest = moInput.getIntake();
        boolean saveSetpoint = moInput.getSaveIntakeSetpoint();

        IntakeSetpoint setpoint = IntakeSetpoint.STOW;

        if (intake.getIsHoldingNote()) {
            setpoint = IntakeSetpoint.HANDOFF;
        }

        if (runIntakeRequest) {
            setpoint = IntakeSetpoint.INTAKE;
        }

        if (Math.abs(deployAdjust) > 0) {
            smartMotionOverride = true;
            smartMotionOverrideTimer.restart();
        } else {
            if (smartMotionOverrideTimer.hasElapsed(INTAKE_OVERRIDE_TIMEOUT)) {
                smartMotionOverride = false;
            }
        }

        Measure<Angle> requestedPos = IntakeSetpointManager.getInstance().getSetpoint(setpoint);

        boolean runIntake = false;

        if (smartMotionOverride) {
            velocityRequest.mut_replace(MoPrefs.intakeDeployMaxSpeed.get());
            velocityRequest.mut_times(deployAdjust);
            intake.deployVelocity(velocityRequest);

            runIntake = runIntakeRequest;

            setpointPublisher.setString("OVERRIDE");
        } else {
            intake.deploySmartMotion(requestedPos);
            runIntake = intake.getDeployPosition().isNear(requestedPos, tolerance);

            setpointPublisher.setString(setpoint.toString());
        }

        runIntakeRollerWithCurrentSense(runIntake);

        if (saveSetpoint) {
            IntakeSetpointManager.getInstance().setSetpoint(setpoint, intake.getDeployPosition());
        }
    }

    private void executeDirectVelocity() {
        var moInput = inputSupplier.get();
        double deployAdjust = moInput.getIntakeAdjust();
        boolean runIntakeRequest = moInput.getIntake();

        velocityRequest.mut_replace(MoPrefs.intakeDeployMaxSpeed.get());
        velocityRequest.mut_times(deployAdjust);
        intake.deployVelocity(velocityRequest);

        runIntakeRollerWithCurrentSense(runIntakeRequest);
    }

    private void executeFallbackDirectPower() {
        var moInput = inputSupplier.get();
        double deployAdjust = moInput.getIntakeAdjust();

        intake.deployFallbackDirectPower(deployAdjust);

        // Note: this is a fallback mode. We can't assume current sense is working. It's up to the driver to ensure they
        // don't shred the notes too much.
        intake.intakeDirectPower(MoPrefs.intakeRollerPower.get().in(Units.Value));
    }

    @Override
    public void execute() {
        var controlMode = intake.controlMode.getSelected();

        if (controlMode != IntakeControlMode.SMARTMOTION) {
            setpointPublisher.setString(controlMode.toString());
        }

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
        smartMotionOverrideTimer.stop();
        currentSenseTimer.stop();
        setpointPublisher.setString("UNKNOWN");
    }
}
