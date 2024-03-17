// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.input;

import frc.robot.component.ArmSetpointManager.ArmSetpoint;
import frc.robot.math.Vec2;
import frc.robot.subsystem.ArmSubsystem.ArmMovementRequest;
import java.util.Optional;

/** Accesses the state of robot input */
public abstract class MoInput {

    public enum ShootTarget {
        SPEAKER,
        AMP,
        SHUTTLE,
        NONE
    };

    protected abstract Optional<ArmSetpoint> getNonShootArmSetpoints();

    public final Optional<ArmSetpoint> getArmSetpoint() {
        var nonShootSetpoint = getNonShootArmSetpoints();
        if (nonShootSetpoint.isPresent()) {
            return nonShootSetpoint;
        }

        var shootTarget = getShootTargetDebounced();
        if (shootTarget == ShootTarget.SPEAKER) {
            return Optional.of(ArmSetpoint.SPEAKER);
        } else if (shootTarget == ShootTarget.AMP) {
            return Optional.of(ArmSetpoint.AMP);
        } else if (shootTarget == ShootTarget.SHUTTLE) {
            return Optional.of(ArmSetpoint.SHUTTLE);
        } else {
            return Optional.empty();
        }
    }

    protected abstract boolean getShoot();

    protected abstract ShootTarget getShootTarget();

    private MoInput.ShootTarget lastTarget = MoInput.ShootTarget.NONE;

    public final ShootTarget getShootTargetDebounced() {
        boolean shoot = getShoot();
        MoInput.ShootTarget curTarget = getShootTarget();

        if (!shoot) {
            lastTarget = MoInput.ShootTarget.NONE;
            return lastTarget;
        }

        if (curTarget != MoInput.ShootTarget.NONE) {
            lastTarget = curTarget;
        }

        return lastTarget;
    }

    public abstract Vec2 getMoveRequest();

    public abstract double getTurnRequest();

    public abstract boolean getShouldUseSlowSpeed();

    public abstract boolean getReZeroGyro();

    public abstract ArmMovementRequest getArmMovementRequest();

    public abstract boolean getSaveArmSetpoint();

    public abstract boolean getReverseShooter();

    public abstract boolean getRunSysId();

    public abstract boolean getReZeroArm();

    public abstract boolean getIntake();

    public abstract boolean getReverseIntake();

    public abstract double getIntakeAdjust();

    public abstract boolean getSaveIntakeSetpoint();

    public abstract boolean getHandoff();

    public abstract double getLeftClimbRequest();

    public abstract double getRightClimbRequest();
}
