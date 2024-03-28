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
import frc.robot.command.arm.MoveArmCommand;
import frc.robot.command.shooter.ShootSpeakerCommand;
import frc.robot.component.ArmSetpointManager.ArmSetpoint;
import frc.robot.util.MoShuffleboard;
import frc.robot.util.PathPlannerCommands;
import java.util.EnumMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.BiFunction;

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

    private enum TaskType {
        LEAVE((s, c) -> c),
        SHOOT((s, c) -> MoveArmCommand.forSetpoint(s.arm, ArmSetpoint.SPEAKER)
                .andThen(new ShootSpeakerCommand(s.shooter).andThen(c)));

        BiFunction<AutoBuilderSubsystem, Command, Command> commandModifier;

        TaskType(BiFunction<AutoBuilderSubsystem, Command, Command> commandModifier) {
            this.commandModifier = commandModifier;
        }
    }

    private SendableChooser<TaskType> taskTypeChooser;
    private SendableChooser<StartPos> startPosChooser;
    private GenericEntry shouldAssumeRobotIsAtStart;
    private GenericEntry masterAutoSwitch;

    private EnumMap<TaskType, EnumMap<StartPos, Optional<PathPlannerPath>>> pathMap;

    private PositioningSubsystem positioning;
    private ArmSubsystem arm;
    private ShooterSubsystem shooter;

    private Optional<PathPlannerPath> tryLoadPath(String path) {
        try {
            return Optional.of(PathPlannerPath.fromPathFile(path));
        } catch (RuntimeException e) {
            DriverStation.reportError(
                    String.format("Error loading path %s: %s", path, e.getLocalizedMessage()), e.getStackTrace());
        }
        return Optional.empty();
    }

    public AutoBuilderSubsystem(PositioningSubsystem positioning, ArmSubsystem arm, ShooterSubsystem shooter) {
        super("Auto Builder");

        this.positioning = positioning;
        this.arm = arm;
        this.shooter = shooter;

        masterAutoSwitch = MoShuffleboard.getInstance()
                .autoTab
                .add("Master Autonomous Switch", true)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .withSize(4, 1)
                .withPosition(0, 0)
                .getEntry();

        taskTypeChooser = MoShuffleboard.enumToChooser(TaskType.class);
        startPosChooser = MoShuffleboard.enumToChooser(StartPos.class);

        var autoPathTypeLayout = MoShuffleboard.getInstance()
                .autoTab
                .getLayout("Auto Path Settings", BuiltInLayouts.kList)
                .withSize(1, 2);

        autoPathTypeLayout.add("Task Type", taskTypeChooser);
        autoPathTypeLayout.add("Start Pos", startPosChooser);

        shouldAssumeRobotIsAtStart = MoShuffleboard.getInstance()
                .autoTab
                .add("Assume Robot at Start Pos?", false)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .withSize(2, 1)
                .withPosition(5, 1)
                .getEntry();

        pathMap = new EnumMap<>(TaskType.class);

        for (var taskType : TaskType.values()) {
            var posMap = new EnumMap<StartPos, Optional<PathPlannerPath>>(StartPos.class);

            for (var startPos : StartPos.values()) {
                posMap.put(startPos, tryLoadPath(String.format("%s %s", taskType.name(), startPos.name())));
            }

            pathMap.put(taskType, posMap);
        }

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
        return pathMap.get(taskTypeChooser.getSelected()).get(startPosChooser.getSelected());
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
            return Commands.print(String.format(
                    "No path defined for %s %s",
                    taskTypeChooser.getSelected().name(),
                    startPosChooser
                            .getSelected()
                            .name())); // "No path defined for starting position " + currStartPos.toString());
        }

        return taskTypeChooser
                .getSelected()
                .commandModifier
                .apply(
                        this,
                        PathPlannerCommands.getFollowPathCommand(
                                drive, positioning, pathToFollow.get(), shouldAssumeRobotIsAtStart.getBoolean(false)));
    }
}
