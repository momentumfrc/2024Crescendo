package frc.robot.command.intake;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.component.IntakeSetpointManager;
import frc.robot.component.IntakeSetpointManager.IntakeSetpoint;
import frc.robot.input.MoInput;
import frc.robot.subsystem.IntakeSubsystem;
import frc.robot.util.MoPrefs;
import frc.robot.util.PositionOverrideUtil;
import java.util.function.Supplier;

public class IdleIntakeCommand extends Command {
    private static final double INTAKE_OVERRIDE_TIMEOUT = 5;

    private final IntakeSubsystem intake;
    private final Supplier<MoInput> inputSupplier;

    private PositionOverrideUtil overrideUtil;

    private MutableMeasure<Distance> rollerPosition = MutableMeasure.zero(Units.Centimeters);

    public IdleIntakeCommand(IntakeSubsystem intake, Supplier<MoInput> inputSupplier) {
        this.intake = intake;
        this.inputSupplier = inputSupplier;

        this.overrideUtil = new PositionOverrideUtil(
                intake::getDeployPosition,
                intake::deploySmartMotion,
                intake::deployVelocity,
                MoPrefs.intakeDeployMaxSpeed,
                INTAKE_OVERRIDE_TIMEOUT);

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        overrideUtil.reset();
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

    @Override
    public void execute() {
        var input = inputSupplier.get();
        double intakeAdjust = input.getIntakeAdjust();

        var controlMode = intake.controlMode.getSelected();

        switch (controlMode) {
            case SMARTMOTION:
                var setpointPos = IntakeSetpointManager.getInstance().getSetpoint(IntakeSetpoint.STOW);
                overrideUtil.runSmartMotionWithAdjust(setpointPos, intakeAdjust);
                break;
            case DIRECT_VELOCITY:
                overrideUtil.runVelocity(intakeAdjust);
                break;
            case FALLBACK_DIRECT_POWER:
                intake.deployFallbackDirectPower(intakeAdjust);
                break;
        }

        if (input.getSaveIntakeSetpoint() && overrideUtil.getInPositionOverride()) {
            IntakeSetpointManager.getInstance().setSetpoint(IntakeSetpoint.STOW, intake.getDeployPosition());
            overrideUtil.reset();
        }

        holdRollerPosition();
    }
}
