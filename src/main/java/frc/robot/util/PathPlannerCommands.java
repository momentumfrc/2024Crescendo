// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.PositioningSubsystem;

public class PathPlannerCommands {
    private static final ReplanningConfig DEFAULT_REPLANNING_CONFIG = new ReplanningConfig(true, false);

    public static Command getFollowPathCommand(
            DriveSubsystem drive,
            PositioningSubsystem positioning,
            PathPlannerPath path,
            boolean shouldAssumeRobotIsAtStart,
            ReplanningConfig replanningConfig) {

        if (replanningConfig.enableInitialReplanning && shouldAssumeRobotIsAtStart) {
            DriverStation.reportWarning("Skipping initial replanning since shouldAssumeRobotIsAtStart is true", false);
            replanningConfig = new ReplanningConfig(
                    false,
                    replanningConfig.enableDynamicReplanning,
                    replanningConfig.dynamicReplanningTotalErrorThreshold,
                    replanningConfig.dynamicReplanningErrorSpikeThreshold);
        }
        // Manually flip the path ourselves, since we need the starting pose
        if (DriverStation.getAlliance().map(a -> a == Alliance.Red).orElse(false)) {
            path = path.flipPath();
        }

        Pose2d startPose = path.getPreviewStartingHolonomicPose();

        double driveBaseRadius = DriveSubsystem.SWERVE_WHEEL_OFFSET.in(Units.Meters) * Math.sqrt(2);

        FollowPathHolonomic driveControllerCommand = new FollowPathHolonomic(
                path,
                positioning::getRobotPose,
                drive::getRobotRelativeSpeeds,
                drive::driveRobotRelativeSpeeds,
                new HolonomicPathFollowerConfig(
                        drive.translationPathController.toImmutable(),
                        drive.rotationPathController.toImmutable(),
                        MoPrefs.autoMaxModuleSpeed.get().in(Units.MetersPerSecond),
                        driveBaseRadius,
                        replanningConfig),
                () -> false, // Do not allow PathPlanner to flip the path, we already did manually
                drive,
                positioning);

        if (shouldAssumeRobotIsAtStart) {
            return new SequentialCommandGroup(
                    new InstantCommand(() -> positioning.setRobotPose(startPose)), driveControllerCommand);
        } else {
            return driveControllerCommand;
        }
    }

    public static Command getFollowPathCommand(
            DriveSubsystem drive,
            PositioningSubsystem positioning,
            String pathName,
            boolean shouldAssumeRobotIsAtStart,
            boolean enableInitialReplanning,
            boolean enableDynamicReplanning) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        return getFollowPathCommand(
                drive,
                positioning,
                path,
                shouldAssumeRobotIsAtStart,
                new ReplanningConfig(enableInitialReplanning, enableDynamicReplanning));
    }

    public static Command getFollowPathCommand(
            DriveSubsystem drive,
            PositioningSubsystem positioning,
            PathPlannerPath path,
            boolean shouldAssumeRobotIsAtStart) {
        return getFollowPathCommand(drive, positioning, path, shouldAssumeRobotIsAtStart, DEFAULT_REPLANNING_CONFIG);
    }

    public static Command getFollowPathCommand(
            DriveSubsystem drive,
            PositioningSubsystem positioning,
            String pathName,
            boolean shouldAssumeRobotIsAtStart) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            return getFollowPathCommand(drive, positioning, path, shouldAssumeRobotIsAtStart);
        } catch (RuntimeException e) {
            DriverStation.reportError("Failed to load autonomous path: " + e.getLocalizedMessage(), e.getStackTrace());
            return Commands.print("Failed to load autonomous path!");
        }
    }
}
