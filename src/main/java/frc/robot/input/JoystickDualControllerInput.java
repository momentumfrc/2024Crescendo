package frc.robot.input;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.component.ArmSetpointManager.ArmSetpoint;
import frc.robot.math.Vec2;
import frc.robot.subsystem.ArmSubsystem.ArmMovementRequest;
import frc.robot.util.MoPrefs;
import frc.robot.util.MoPrefs.Pref;
import frc.robot.util.MoUtils;
import java.util.Optional;

public class JoystickDualControllerInput extends MoInput {
    private final Joystick joystick;
    private final XboxController armController;

    private Pref<Double> driveDeadzone = MoPrefs.driveDeadzone;
    private Pref<Double> driveCurve = MoPrefs.driveCurve;

    public JoystickDualControllerInput(Constants.HIDPort driveStickPort, Constants.HIDPort armControllerPort) {
        joystick = new Joystick(driveStickPort.port());
        armController = new XboxController(armControllerPort.port());
    }

    private double applyInputTransforms(double value) {
        return MoUtils.curve(MoUtils.deadzone(value, driveDeadzone.get()), driveCurve.get());
    }

    private double getThrottle() {
        return ((-1 * joystick.getThrottle()) + 1) / 2;
    }

    @Override
    public Vec2 getMoveRequest() {
        return new Vec2(joystick.getX(), joystick.getY())
                .scalarOp((v) -> v *= getThrottle())
                .scalarOp(this::applyInputTransforms);
    }

    @Override
    public double getTurnRequest() {
        return MoUtils.curve(
                MoUtils.deadzone(-1 * joystick.getZ() * getThrottle(), MoPrefs.driveTurnDeadzone.get()),
                MoPrefs.driveTurnCurve.get());
    }

    @Override
    public boolean driveRobotOriented() {
        return joystick.getRawButton(2);
    }

    @Override
    public boolean getShouldUseSlowSpeed() {
        // No explicit slow speed since we have the throttle.
        return false;
    }

    @Override
    public boolean getReZeroGyro() {
        return joystick.getRawButton(7);
    }

    @Override
    public ArmMovementRequest getArmMovementRequest() {
        return new ArmMovementRequest(
                applyInputTransforms(armController.getLeftY()), applyInputTransforms(armController.getRightY()));
    }

    @Override
    public Optional<ArmSetpoint> getNonShootArmSetpoints() {
        if (armController.getBButton()) {
            return Optional.of(ArmSetpoint.STOW);
        } else if (armController.getRightBumper()) {
            return Optional.of(ArmSetpoint.HANDOFF);
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
        return joystick.getRawButton(12);
    }

    @Override
    public boolean getReZeroArm() {
        return armController.getStartButton();
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
        } else if (pov == 90) {
            return MoInput.ShootTarget.SHUTTLE;
        } else {
            return MoInput.ShootTarget.NONE;
        }
    }

    public boolean getReverseShooter() {
        return armController.getXButton() && armController.getAButton();
    }

    public boolean getReverseIntake() {
        return joystick.getRawButton(3);
    }

    @Override
    public boolean getIntake() {
        return joystick.getRawButton(1);
    }

    @Override
    public double getIntakeAdjust() {
        double value = 0;
        int pov = joystick.getPOV();

        if (pov == 0) {
            value = 1;
        } else if (pov == 180) {
            value = -1;
        }

        value *= MoPrefs.intakeAdjustPower.get().in(Units.Value);

        return value;
    }

    @Override
    public boolean getSaveIntakeSetpoint() {
        return joystick.getRawButton(8);
    }

    @Override
    public boolean getHandoff() {
        return armController.getRightBumper();
    }

    private boolean getClimbReverse() {
        return armController.getLeftBumper();
    }

    @Override
    public double getLeftClimbRequest() {
        return (getClimbReverse() ? -1 : 1) * armController.getLeftTriggerAxis();
    }

    @Override
    public double getRightClimbRequest() {
        return (getClimbReverse() ? -1 : 1) * armController.getRightTriggerAxis();
    }

    @Override
    public boolean getShouldTargetNote() {
        return joystick.getRawButton(4);
    }
}
