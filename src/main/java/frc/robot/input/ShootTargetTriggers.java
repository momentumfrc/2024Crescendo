package frc.robot.input;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class ShootTargetTriggers {
    private Supplier<MoInput> inputSupplier;

    private MoInput.ShootTarget target = MoInput.ShootTarget.NONE;

    public ShootTargetTriggers(Supplier<MoInput> inputSupplier) {
        this.inputSupplier = inputSupplier;
    }

    public BooleanSupplier getTriggerForShootTarget(MoInput.ShootTarget forTarget) {
        return () -> {
            var input = inputSupplier.get();
            boolean shoot = input.getShoot();
            MoInput.ShootTarget curTarget = input.getShootTarget();

            if (!shoot) {
                target = MoInput.ShootTarget.NONE;
                return false;
            }

            if (curTarget != MoInput.ShootTarget.NONE) {
                target = curTarget;
            }

            return forTarget == target;
        };
    }
}
