package frc.robot.subsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.command.arm.MoveArmCommand;
import frc.robot.command.shooter.ShootSpeakerCommand;
import frc.robot.component.ArmSetpointManager.ArmSetpoint;
import frc.robot.util.MoShuffleboard;
import java.util.Map;

public class AutoBuilderSubsystem extends SubsystemBase {
    private static final ReplanningConfig DEFAULT_REPLANNING_CONFIG = new ReplanningConfig(false, false);

    // All poses are against the blue alliance back wall
    private enum StartPosePreset {
        AMP(0.53, 7.00, 0), // Lined up with the alliance-side note closest to the judges' side of the field
        CENTER(0.53, 4.08, 0), // Lined up with the alliance-side note in front of the stage's front leg
        SOURCE(0.53, 2.41, 0), // Lined up with the fourth center-field note away from the judges
        SPEAKER_AMP_SIDE(0.73, 6.70, 60),
        SPEAKER_CENTER(1.38, 5.54, 0),
        SPEAKER_SOURCE_SIDE(0.73, 4.39, -60);

        Pose2d pose;

        /**
         * Get these values from PathPlanner's preview start pose, when making a path
         */
        StartPosePreset(double x, double y, double heading) {
            this.pose = new Pose2d(Units.Meters.of(x), Units.Meters.of(y), new Rotation2d(Units.Degrees.of(heading)));
        }
    };

    private GenericEntry masterAutoSwitch;
    private GenericEntry overridePoseSwitch;
    private SendableChooser<Command> autoChooser;
    private SendableChooser<StartPosePreset> posePresetChooser;
    private Pose2d startPose = new Pose2d(0, 0, new Rotation2d(0));

    public AutoBuilderSubsystem(PositioningSubsystem positioning, DriveSubsystem drive) {
        super("Auto Builder");

        var autoTab = MoShuffleboard.getInstance().autoTab;

        masterAutoSwitch = autoTab.add("Master Autonomous Switch", true)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .withSize(3, 1)
                .withPosition(0, 0)
                .getEntry();

        overridePoseSwitch = autoTab.add("Override Start Pose?", true)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .withSize(1, 1)
                .withPosition(1, 0)
                .getEntry();

        autoChooser = AutoBuilder.buildAutoChooser();
        autoTab.add("PP Auto", autoChooser).withSize(2, 1).withPosition(1, 1);

        posePresetChooser = MoShuffleboard.enumToChooser(StartPosePreset.class);
        posePresetChooser.onChange(preset -> this.flipAndSetStartPose(preset.pose));
        autoTab.add("Start Pose Preset", posePresetChooser)
                .withSize(1, 2)
                .withPosition(4, 0)
                .withWidget(BuiltInWidgets.kSplitButtonChooser);

        var poseViewLayout = autoTab.getLayout("Start Pose Override", BuiltInLayouts.kList)
                .withProperties(Map.of("Label position", "LEFT"))
                .withSize(2, 2)
                .withPosition(5, 0);
        poseViewLayout.addDouble("X", () -> this.startPose.getX());
        poseViewLayout.addDouble("Y", () -> this.startPose.getY());
        poseViewLayout.addDouble("Rot", () -> this.startPose.getRotation().getDegrees());

        AutoBuilder.configureRamsete(
                positioning::getRobotPose,
                positioning::setRobotPose,
                drive::getRobotRelativeSpeeds,
                drive::driveRobotRelativeSpeeds,
                DEFAULT_REPLANNING_CONFIG,
                AutoBuilderSubsystem::flipPath,
                drive);
    }

    public static boolean flipPath() {
        return DriverStation.getAlliance().map(a -> a == Alliance.Red).orElse(false);
    }

    private void flipAndSetStartPose(Pose2d pose) {
        this.startPose = flipPath() ? GeometryUtil.flipFieldPose(pose) : pose;
    }

    public Command getAutonomousCommand(PositioningSubsystem positioning) {
        if (!masterAutoSwitch.getBoolean(true)) {
            return Commands.print("Autonomous disabled via shuffleboard");
        }

        if (overridePoseSwitch.getBoolean(true)) {
            return new InstantCommand(() -> positioning.setRobotPose(this.startPose))
                    .andThen(autoChooser.getSelected());
        }

        return autoChooser.getSelected();
    }
}
