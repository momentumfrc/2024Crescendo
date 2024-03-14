// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.input;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.component.ArmSetpointManager.ArmSetpoint;
import frc.robot.math.Vec2;
import frc.robot.subsystem.ArmSubsystem.ArmMovementRequest;
import frc.robot.util.MoPrefs;
import frc.robot.util.MoPrefs.Pref;
import frc.robot.util.MoUtils;
import java.util.Optional;

public class SingleControllerInput implements MoInput {
    private final XboxController controller;

    private Pref<Double> driveDeadzone = MoPrefs.driveDeadzone;
    private Pref<Double> driveCurve = MoPrefs.driveCurve;

    public SingleControllerInput(Constants.HIDPort port) {
        this.controller = new XboxController(port.port());
    }

    private double applyDriveInputTransforms(double value) {
        return MoUtils.curve(MoUtils.deadzone(value, driveDeadzone.get()), driveCurve.get());
    }

    @Override
    public Vec2 getMoveRequest() {
        return new Vec2(controller.getLeftX(), controller.getLeftY()).scalarOp(this::applyDriveInputTransforms);
    }

    @Override
    public double getTurnRequest() {
        return -1 * applyDriveInputTransforms(controller.getRightX());
    }

    @Override
    public boolean getShouldUseSlowSpeed() {
        return controller.getLeftTriggerAxis() > 0.8;
    }

    @Override
    public boolean getReZeroGyro() {
        return controller.getStartButton();
    }

    @Override
    public ArmMovementRequest getArmMovementRequest() {
        // Speed overrides not supported in single-controller mode
        return new ArmMovementRequest(0, 0);
    }

    @Override
    public Optional<ArmSetpoint> getArmSetpoint() {
        if (controller.getBButton()) {
            return Optional.of(ArmSetpoint.STOW);
        } else if (controller.getAButton()) {
            return Optional.of(ArmSetpoint.HANDOFF);
        } else if (controller.getXButton()) {
            double pov = controller.getPOV();
            if (pov == 180) {
                return Optional.of(ArmSetpoint.AMP);
            }
        } else if (controller.getYButton()) {
            return Optional.of(ArmSetpoint.SOURCE);
        }

        return Optional.empty();
    }

    @Override
    public boolean getSaveArmSetpoint() {
        // Overwriting setpoints is not supported in single-controller mode
        return false;
    }

    @Override
    public boolean getRunSysId() {
        return controller.getLeftBumper();
    }

    @Override
    public boolean getShoot() {
        return controller.getXButton();
    }

    @Override
    public MoInput.ShootTarget getShootTarget() {
        int pov = controller.getPOV();
        if (pov == 0) {
            return MoInput.ShootTarget.SPEAKER;
        } else if (pov == 180) {
            return MoInput.ShootTarget.AMP;
        } else {
            return MoInput.ShootTarget.NONE;
        }
    }

    public boolean getReverseShooter() {
        // Not supported in single-controller mode
        return false;
    }

    public boolean getReverseIntake() {
        // Not supported in single-controller mode
        return false;
    }

    @Override
    public boolean getReZeroArm() {
        // Re-zeroing is not supported in single-controller mode
        return false;
    }

    @Override
    public boolean getIntake() {
        return controller.getLeftBumper();
    }

    @Override
    public double getIntakeAdjust() {
        // Speed overrides is not supported in single-controller mode
        return 0;
    }

    @Override
    public boolean getSaveIntakeSetpoint() {
        // Overwriting setpoints is not supported in single-controller mode
        return false;
    }

    @Override
    public boolean getHandoff() {
        return controller.getAButton();
    }
}
