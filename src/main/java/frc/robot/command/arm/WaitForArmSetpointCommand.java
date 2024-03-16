package frc.robot.command.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.component.ArmSetpointManager;
import frc.robot.component.ArmSetpointManager.ArmSetpoint;
import frc.robot.subsystem.ArmSubsystem;

public class WaitForArmSetpointCommand extends Command {
    private final ArmSubsystem arm;
    private final ArmSetpoint waitForSetpoint;

    public WaitForArmSetpointCommand(ArmSubsystem arm, ArmSetpoint waitForSetpoint) {
        this.arm = arm;
        this.waitForSetpoint = waitForSetpoint;
    }

    @Override
    public boolean isFinished() {
        var waitForPos = ArmSetpointManager.getInstance().getSetpoint(waitForSetpoint);
        return arm.atPosition(waitForPos);
    }
}
