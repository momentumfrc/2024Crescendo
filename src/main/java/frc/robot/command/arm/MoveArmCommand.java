package frc.robot.command.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.component.ArmSetpointManager;
import frc.robot.component.ArmSetpointManager.ArmSetpoint;
import frc.robot.subsystem.ArmSubsystem;
import frc.robot.subsystem.ArmSubsystem.ArmPosition;

public class MoveArmCommand extends Command {
    private final ArmSubsystem arm;
    private final ArmPosition position;

    public MoveArmCommand(ArmSubsystem arm, ArmPosition toPosition) {
        this.arm = arm;
        this.position = toPosition;

        addRequirements(arm);
    }

    public static MoveArmCommand forSetpoint(ArmSubsystem arm, ArmSetpoint setpoint) {
        return new MoveArmCommand(arm, ArmSetpointManager.getInstance().getSetpoint(setpoint));
    }

    @Override
    public void execute() {
        arm.adjustSmartPosition(position);
    }

    public boolean onTarget() {
        return arm.atPosition(position);
    }
}
