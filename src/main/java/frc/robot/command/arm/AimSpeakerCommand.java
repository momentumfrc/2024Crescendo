package frc.robot.command.arm;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.component.ArmSetpointManager;
import frc.robot.component.ArmSetpointManager.ArmSetpoint;
import frc.robot.subsystem.ArmSubsystem;
import frc.robot.subsystem.ArmSubsystem.ArmPosition;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.PositioningSubsystem;
import frc.robot.util.MoPrefs;
import frc.robot.util.TargetAngleFinder;
import java.util.EnumMap;

public class AimSpeakerCommand extends Command {
    private final ArmSubsystem arm;
    private final DriveSubsystem drive;
    private final PositioningSubsystem pos;

    private final TargetAngleFinder targeting = TargetAngleFinder.getInstance();

    private EnumMap<DriverStation.Alliance, Pose2d> speakerPoses = new EnumMap<>(DriverStation.Alliance.class);

    private MutableMeasure<Distance> mutDist = MutableMeasure.zero(Units.Meters);
    private MutableMeasure<Angle> mutAngle = MutableMeasure.zero(Units.Rotations);

    ArmPosition adjustedPosition;
    Measure<Angle> headingOffset;

    public AimSpeakerCommand(ArmSubsystem arm, DriveSubsystem drive, PositioningSubsystem pos) {
        this.arm = arm;
        this.drive = drive;
        this.pos = pos;

        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
        speakerPoses.put(DriverStation.Alliance.Blue, layout.getTagPose(7).get().toPose2d());
        speakerPoses.put(DriverStation.Alliance.Red, layout.getTagPose(4).get().toPose2d());

        addRequirements(drive, arm);
    }

    private final Pose2d getSpeakerPose() {
        return speakerPoses.get(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue));
    }

    @Override
    public void execute() {
        Pose2d robotPose = pos.getAbsoluteRobotPose();
        Pose2d speakerPose = getSpeakerPose();

        Transform2d transform = speakerPose.minus(robotPose);

        Measure<Angle> wristAim = targeting.getWristAngle(
                mutDist.mut_replace(transform.getTranslation().getNorm(), Units.Meters));
        headingOffset = mutAngle.mut_replace(transform.getRotation().getRadians(), Units.Radians);

        ArmPosition aimPosition = ArmSetpointManager.getInstance().getSetpoint(ArmSetpoint.SPEAKER);
        adjustedPosition = new ArmPosition(aimPosition.shoulderAngle(), wristAim);

        arm.adjustSmartPosition(adjustedPosition);
        drive.rotateRelative(new Rotation2d(headingOffset));
    }

    public boolean onTarget() {
        if (headingOffset == null || adjustedPosition == null) {
            return false;
        }

        double thresh = MoPrefs.pidSetpointVarianceThreshold.get().in(Units.Value);
        return headingOffset.isNear(Units.Rotations.zero(), thresh) && arm.atPosition(adjustedPosition, thresh);
    }
}
