package frc.robot.command.arm;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.component.ArmSetpointManager;
import frc.robot.component.ArmSetpointManager.ArmSetpoint;
import frc.robot.input.MoInput;
import frc.robot.subsystem.ArmSubsystem;
import frc.robot.subsystem.ArmSubsystem.ArmMovementRequest;
import frc.robot.subsystem.ArmSubsystem.ArmPosition;
import frc.robot.util.MoPrefs;
import java.util.Optional;
import java.util.function.Supplier;

public class TeleopArmCommand extends Command {
    private static final ArmSetpoint DEFAULT_SETPOINT = ArmSetpoint.STOW;

    private final ArmSubsystem arms;
    private final Supplier<MoInput> inputSupplier;

    private boolean smartMotionOverride = false;

    private SlewRateLimiter shoulderLimiter;
    private SlewRateLimiter wristLimiter;

    public TeleopArmCommand(ArmSubsystem arms, Supplier<MoInput> inputSupplier) {
        this.arms = arms;
        this.inputSupplier = inputSupplier;

        MoPrefs.armRampTime.subscribe(
                rampTime -> {
                    double slewRate = 1.0 / rampTime;

                    shoulderLimiter = new SlewRateLimiter(slewRate);
                    wristLimiter = new SlewRateLimiter(slewRate);
                },
                true);

        addRequirements(arms);
    }

    private ArmMovementRequest getMovementRequest(MoInput input) {
        ArmMovementRequest requestedMovement = input.getArmMovementRequest();
        return new ArmMovementRequest(
                shoulderLimiter.calculate(requestedMovement.shoulderPower()),
                wristLimiter.calculate(requestedMovement.wristPower()));
    }

    private void moveSmartMotion(MoInput input) {
        Optional<ArmSetpoint> requestedSetpoint = input.getArmSetpoint();
        ArmMovementRequest requestedMovement = getMovementRequest(input);
        boolean shouldSaveSetpoint = input.getSaveArmSetpoint();

        if (requestedSetpoint.isPresent()) {
            if (shouldSaveSetpoint) {
                ArmSetpointManager.getInstance().setSetpoint(requestedSetpoint.get(), arms.getArmPosition());
            } else {
                smartMotionOverride = false;
            }
        }

        if (!requestedMovement.isZero()) {
            smartMotionOverride = true;
        }

        ArmSetpoint setpoint = requestedSetpoint.orElse(DEFAULT_SETPOINT);
        ArmPosition requestedPosition = ArmSetpointManager.getInstance().getSetpoint(setpoint);
        if (smartMotionOverride) {
            arms.adjustVelocity(requestedMovement);
        } else {
            arms.adjustSmartPosition(requestedPosition);
        }
    }

    @Override
    public void execute() {
        MoInput input = inputSupplier.get();
        var controlMode = arms.controlMode.getSelected();

        switch (controlMode) {
            case FALLBACK_DIRECT_POWER:
                arms.adjustDirectPower(getMovementRequest(input));
                return;
            case DIRECT_VELOCITY:
                arms.adjustVelocity(getMovementRequest(input));
                return;
            case SMARTMOTION:
                moveSmartMotion(input);
                return;
        }
    }
}
