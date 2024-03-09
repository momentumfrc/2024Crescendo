package frc.robot.subsystem;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MoShuffleboard;
import frc.robot.util.PathPlannerCommands;
import java.util.EnumMap;
import java.util.Map;
import java.util.Optional;

public class AutoBuilderSubsystem extends SubsystemBase {
    /**
     * If the robot is within this distance of the starting point of any configured path, it will try to use that path
     * for autonomous.
     */
    private static final Measure<Distance> INITIAL_POSE_RANGE = Units.Meters.of(2);

    private enum StartPos {
        LEFT,
        CENTER,
        RIGHT
    };

    private SendableChooser<StartPos> startPosChooser;
    private GenericEntry shouldAssumeRobotIsAtStart;
    private GenericEntry masterAutoSwitch;

    private EnumMap<StartPos, Optional<PathPlannerPath>> leavePathMap;

    private PositioningSubsystem positioning;

    private StartPos currStartPos = StartPos.LEFT;

    private Optional<PathPlannerPath> tryLoadPath(String path) {
        try {
            return Optional.of(PathPlannerPath.fromPathFile(path));
        } catch (RuntimeException e) {
            DriverStation.reportError(
                    String.format("Error loading path %s: %s", path, e.getLocalizedMessage()), e.getStackTrace());
        }
        return Optional.empty();
    }

    public AutoBuilderSubsystem(PositioningSubsystem positioning) {
        super("Auto Builder");

        this.positioning = positioning;

        masterAutoSwitch = MoShuffleboard.getInstance()
                .autoTab
                .add("Master Autonomous Switch", true)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .withSize(4, 1)
                .withPosition(0, 0)
                .getEntry();

        startPosChooser = MoShuffleboard.enumToChooser(StartPos.class);
        MoShuffleboard.getInstance()
                .autoTab
                .add("Start Pos Override", startPosChooser)
                .withSize(1, 1)
                .withPosition(3, 1);

        shouldAssumeRobotIsAtStart = MoShuffleboard.getInstance()
                .autoTab
                .add("Assume Robot at Start Pos?", false)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .withSize(2, 1)
                .withPosition(1, 1)
                .getEntry();

        leavePathMap = new EnumMap<>(StartPos.class);
        leavePathMap.put(StartPos.LEFT, tryLoadPath("LEAVE LEFT"));
        leavePathMap.put(StartPos.CENTER, tryLoadPath("LEAVE CENTER"));
        leavePathMap.put(StartPos.RIGHT, tryLoadPath("LEAVE RIGHT"));

        MoShuffleboard.getInstance()
                .autoTab
                .addString("Curr Start Pos", () -> this.currStartPos.toString())
                .withSize(1, 1)
                .withPosition(0, 1);

        var alignmentGroup = MoShuffleboard.getInstance()
                .autoTab
                .getLayout("Starting Position Alignment", BuiltInLayouts.kList)
                .withSize(2, 2)
                .withPosition(0, 2)
                .withProperties(Map.of("Label position", "LEFT"));
        alignmentGroup.addDouble("Distance", () -> getStartingPose()
                .map(pose -> positioning.getRobotPose().getTranslation().getDistance(pose.getTranslation()))
                .orElse(0.0));
        alignmentGroup.addDouble("X", () -> getStartingPose()
                .map(pose -> pose.getTranslation().getX()
                        - positioning.getRobotPose().getTranslation().getX())
                .orElse(0.0));
        alignmentGroup.addDouble("Y", () -> getStartingPose()
                .map(pose -> pose.getTranslation().getY()
                        - positioning.getRobotPose().getY())
                .orElse(0.0));
    }

    private Optional<PathPlannerPath> getPathToFollow() {
        return leavePathMap.get(currStartPos);
    }

    private Optional<Pose2d> getStartingPose() {
        return getPathToFollow().map(curr -> {
            PathPlannerPath path = curr;
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                path = path.flipPath();
            }

            return path.getPreviewStartingHolonomicPose();
        });
    }

    public Command getAutonomousCommand(DriveSubsystem drive) {
        if (!masterAutoSwitch.getBoolean(true)) {
            return Commands.print("Autonomous disabled via shuffleboard");
        }

        Optional<PathPlannerPath> pathToFollow = getPathToFollow();
        if (pathToFollow.isEmpty()) {
            return Commands.print("No path defined for starting position " + currStartPos.toString());
        }

        return PathPlannerCommands.getFollowPathCommand(
                drive, positioning, pathToFollow.get(), shouldAssumeRobotIsAtStart.getBoolean(false));
    }

    @Override
    public void periodic() {
        if (!shouldAssumeRobotIsAtStart.getBoolean(false) && positioning.hasInitialPosition()) {
            StartPos currStartPos = null;
            double currSmallestDist = Double.POSITIVE_INFINITY;
            Pose2d robotPose = positioning.getRobotPose();
            for (StartPos pos : StartPos.values()) {
                if (!leavePathMap.containsKey(pos) || leavePathMap.get(pos).isEmpty()) {
                    continue;
                }

                PathPlannerPath path = leavePathMap.get(pos).get();
                if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                    path = path.flipPath();
                }

                Pose2d posStartPose = path.getPreviewStartingHolonomicPose();
                double dist = posStartPose.getTranslation().getDistance(robotPose.getTranslation());
                if (currStartPos == null || dist < currSmallestDist) {
                    currStartPos = pos;
                    currSmallestDist = dist;
                }
            }
        } else {
            currStartPos = startPosChooser.getSelected();
        }
    }
}
