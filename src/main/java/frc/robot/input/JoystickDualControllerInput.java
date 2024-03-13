package frc.robot.input;

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

public class JoystickDualControllerInput implements MoInput {
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
        return applyInputTransforms(-1 * joystick.getZ() * getThrottle());
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
    public Optional<ArmSetpoint> getArmSetpoint() {
        if (armController.getBButton()) {
            return Optional.of(ArmSetpoint.STOW);
        } else if (armController.getAButton()) {
            return Optional.of(ArmSetpoint.HANDOFF);
        } else if (armController.getXButton()) {
            double pov = armController.getPOV();
            if (pov == 0) {
                return Optional.of(ArmSetpoint.SPEAKER);
            } else if (pov == 180) {
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
    public boolean getReZeroArm() {
        return armController.getStartButton();
    }

    @Override
    public boolean getShouldShootSpeaker() {
        return armController.getXButton() && armController.getPOV() == 0;
    }

    @Override
    public boolean getShouldShootAmp() {
        return armController.getXButton() && armController.getPOV() == 180;
    }

    @Override
    public boolean getIntake() {
        return joystick.getRawButton(3);
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

        value *= joystick.getThrottle();

        return value;
    }

    @Override
    public boolean getSaveIntakeSetpoint() {
        return joystick.getRawButton(8);
    }

    @Override
    public boolean getHandoff() {
        return armController.getAButton();
    }
}
