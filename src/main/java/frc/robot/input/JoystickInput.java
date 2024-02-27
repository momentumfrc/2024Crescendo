package frc.robot.input;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import frc.robot.math.Vec2;
import frc.robot.util.MoPrefs;
import frc.robot.util.MoPrefs.Pref;
import frc.robot.util.MoUtils;

public class JoystickInput implements MoInput {
    private final Joystick joystick;

    private Pref<Double> driveDeadzone = MoPrefs.driveDeadzone;
    private Pref<Double> driveCurve = MoPrefs.driveCurve;

    public JoystickInput(Constants.HIDPort port) {
        joystick = new Joystick(port.port());
    }

    private double applyDriveInputTransforms(double value) {
        return MoUtils.curve(MoUtils.deadzone(value, driveDeadzone.get()), driveCurve.get());
    }

    private double getThrottle() {
        return ((-1 * joystick.getThrottle()) + 1) / 2;
    }

    @Override
    public Vec2 getMoveRequest() {
        return new Vec2(joystick.getX(), joystick.getY())
                .scalarOp((v) -> v *= getThrottle())
                .scalarOp(this::applyDriveInputTransforms);
    }

    @Override
    public double getTurnRequest() {
        return applyDriveInputTransforms(joystick.getZ() * getThrottle());
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
}
