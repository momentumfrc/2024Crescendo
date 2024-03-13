package frc.robot.command.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.component.ArmSetpointManager;
import frc.robot.component.ArmSetpointManager.ArmSetpoint;
import frc.robot.subsystem.ArmSubsystem;
import frc.robot.subsystem.ArmSubsystem.ArmPosition;
import frc.robot.subsystem.PositioningSubsystem;
import frc.robot.util.TargetAngleFinder;

public class AimSpeakerCommand extends Command {
    private final ArmSubsystem arm;
    private final PositioningSubsystem pos;

    private final TargetAngleFinder targeting = TargetAngleFinder.getInstance();

    private MutableMeasure<Distance> mutDist = MutableMeasure.zero(Units.Meters);

    private ArmPosition adjustedPosition;

    public AimSpeakerCommand(ArmSubsystem arm, PositioningSubsystem pos) {
        this.arm = arm;
        this.pos = pos;

        addRequirements(arm);
    }

    @Override
    public void execute() {
        Pose2d robotPose = pos.getRobotPose();
        Pose2d speakerPose = pos.getSpeakerPose();

        Transform2d transform = speakerPose.minus(robotPose);

        Measure<Angle> wristAim = targeting.getWristAngle(
                mutDist.mut_replace(transform.getTranslation().getNorm(), Units.Meters));

        ArmPosition aimPosition = ArmSetpointManager.getInstance().getSetpoint(ArmSetpoint.SPEAKER);
        adjustedPosition = new ArmPosition(aimPosition.shoulderAngle(), wristAim);

        arm.adjustSmartPosition(adjustedPosition);
    }

    public boolean onTarget() {
        if (adjustedPosition == null) {
            return false;
        }

        return arm.atPosition(adjustedPosition);
    }
}
