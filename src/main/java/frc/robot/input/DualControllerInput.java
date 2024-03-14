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

public class DualControllerInput implements MoInput {
    private final XboxController driveController;
    private final XboxController armController;

    private Pref<Double> driveDeadzone = MoPrefs.driveDeadzone;
    private Pref<Double> driveCurve = MoPrefs.driveCurve;

    public DualControllerInput(Constants.HIDPort drivePort, Constants.HIDPort armPort) {
        this.driveController = new XboxController(drivePort.port());
        this.armController = new XboxController(armPort.port());
    }

    private double applyInputTransforms(double value) {
        return MoUtils.curve(MoUtils.deadzone(value, driveDeadzone.get()), driveCurve.get());
    }

    @Override
    public Vec2 getMoveRequest() {
        return new Vec2(driveController.getLeftX(), driveController.getLeftY()).scalarOp(this::applyInputTransforms);
    }

    @Override
    public double getTurnRequest() {
        return -1 * applyInputTransforms(driveController.getRightX());
    }

    @Override
    public boolean getShouldUseSlowSpeed() {
        return driveController.getLeftBumper();
    }

    @Override
    public boolean getReZeroGyro() {
        return driveController.getStartButton();
    }

    @Override
    public ArmMovementRequest getArmMovementRequest() {
        return new ArmMovementRequest(
                applyInputTransforms(armController.getLeftY()), applyInputTransforms(armController.getRightY()));
    }

    @Override
    public Optional<ArmSetpoint> getArmSetpoint() {
        if (armController.getBButton()) {
            return Optional.of(ArmSetpoint.STOW);
        } else if (armController.getAButton()) {
            return Optional.of(ArmSetpoint.HANDOFF);
        } else if (armController.getXButton()) {
            double pov = armController.getPOV();
            if (pov == 180) {
                return Optional.of(ArmSetpoint.AMP);
            }
        } else if (armController.getYButton()) {
            return Optional.of(ArmSetpoint.SOURCE);
        }

        return Optional.empty();
    }

    @Override
    public boolean getSaveArmSetpoint() {
        return armController.getBackButton();
    }

    @Override
    public boolean getRunSysId() {
        return armController.getLeftBumper();
    }

    @Override
    public boolean getShoot() {
        return armController.getXButton();
    }

    @Override
    public MoInput.ShootTarget getShootTarget() {
        int pov = armController.getPOV();
        if (pov == 0) {
            return MoInput.ShootTarget.SPEAKER;
        } else if (pov == 180) {
            return MoInput.ShootTarget.AMP;
        } else {
            return MoInput.ShootTarget.NONE;
        }
    }

    public boolean getReverseShooter() {
        return armController.getXButton() && armController.getRightBumper();
    }

    public boolean getReverseIntake() {
        return armController.getLeftBumper() && armController.getRightBumper();
    }

    @Override
    public boolean getReZeroArm() {
        return armController.getStartButton();
    }

    @Override
    public boolean getIntake() {
        return driveController.getRightBumper();
    }

    @Override
    public double getIntakeAdjust() {
        return applyInputTransforms(driveController.getRightY());
    }

    @Override
    public boolean getSaveIntakeSetpoint() {
        return driveController.getBackButton();
    }

    @Override
    public boolean getHandoff() {
        return armController.getAButton();
    }
}
