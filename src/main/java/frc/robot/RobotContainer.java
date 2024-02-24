// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.command.AimSpeakerCommand;
import frc.robot.command.CalibrateSwerveDriveCommand;
import frc.robot.command.CalibrateSwerveTurnCommand;
import frc.robot.command.CoastSwerveDriveCommand;
import frc.robot.command.OrchestraCommand;
import frc.robot.command.TeleopArmCommand;
import frc.robot.command.TeleopDriveCommand;
import frc.robot.component.ArmSetpointManager.ArmSetpoint;
import frc.robot.input.DualControllerInput;
import frc.robot.input.MoInput;
import frc.robot.input.SingleControllerInput;
import frc.robot.subsystem.ArmSubsystem;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.PositioningSubsystem;
import frc.robot.util.MoShuffleboard;
import frc.robot.util.PathPlannerCommands;
import java.util.Set;

public class RobotContainer {
    private enum SysIdMode {
        NONE,
        QUASISTATIC_FORWARD,
        QUASISTATIC_REVERSE,
        DYNAMIC_FORWARD,
        DYNAMIC_REVERSE
    };

    private AHRS gyro = new AHRS(SerialPort.Port.kMXP);

    // Subsystems
    private DriveSubsystem drive = new DriveSubsystem(gyro);
    private PositioningSubsystem positioning = new PositioningSubsystem(gyro, drive);
    private ArmSubsystem arm = new ArmSubsystem();

    // Commands
    private TeleopDriveCommand driveCommand = new TeleopDriveCommand(drive, positioning, this::getInput);
    private TeleopArmCommand armCommand = new TeleopArmCommand(arm, this::getInput);
    private OrchestraCommand startupOrchestraCommand = new OrchestraCommand(drive, this::getInput, "windows-xp.chrp");

    private SendableChooser<MoInput> inputChooser = new SendableChooser<>();
    private final StringEntry autoPathEntry;
    private final BooleanEntry autoAssumeAtStartEntry;

    private SendableChooser<SysIdMode> sysidMode = MoShuffleboard.enumToChooser(SysIdMode.class);

    private final NetworkButton calibrateDriveButton;
    private final NetworkButton calibrateTurnButton;
    private final NetworkButton coastSwerveButton;

    private final Trigger runSysidTrigger;
    private final Trigger aimSpeakerTrigger;

    private final GenericEntry shouldPlayEnableTone = MoShuffleboard.getInstance()
            .settingsTab
            .add("Tone on Enable", false)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();

    public RobotContainer() {
        inputChooser.setDefaultOption(
                "Dual Controller", new DualControllerInput(Constants.DRIVE_F310, Constants.ARM_F310));
        inputChooser.addOption("Single Controller", new SingleControllerInput(Constants.DRIVE_F310));

        MoShuffleboard.getInstance().settingsTab.add("Controller Mode", inputChooser);
        MoShuffleboard.getInstance().settingsTab.add("Sysid Mode", sysidMode);

        BooleanEntry calibrateDriveEntry = NetworkTableInstance.getDefault()
                .getTable("Settings")
                .getBooleanTopic("Calibrate Drive Encoders")
                .getEntry(false);
        calibrateDriveEntry.setDefault(false);
        calibrateDriveButton = new NetworkButton(calibrateDriveEntry);

        BooleanEntry calibrateTurnEntry = NetworkTableInstance.getDefault()
                .getTable("Settings")
                .getBooleanTopic("Calibrate Turn Encoders")
                .getEntry(false);
        calibrateTurnEntry.setDefault(false);
        calibrateTurnButton = new NetworkButton(calibrateTurnEntry);

        BooleanEntry coastSwerveEntry = NetworkTableInstance.getDefault()
                .getTable("Settings")
                .getBooleanTopic("Coast Swerve Drive")
                .getEntry(false);
        coastSwerveEntry.setDefault(false);
        coastSwerveButton = new NetworkButton(coastSwerveEntry);

        this.autoPathEntry = NetworkTableInstance.getDefault()
                .getTable("Settings")
                .getStringTopic("Autonomous Path")
                .getEntry("");
        this.autoAssumeAtStartEntry = NetworkTableInstance.getDefault()
                .getTable("Settings")
                .getBooleanTopic("Auto Assume At Start")
                .getEntry(true);
        runSysidTrigger = new Trigger(() -> getInput().getRunSysId());
        aimSpeakerTrigger = new Trigger(() -> getInput()
                .getArmSetpoint()
                .map((setpoint) -> setpoint == ArmSetpoint.SPEAKER)
                .orElse(false));

        drive.setDefaultCommand(driveCommand);
        arm.setDefaultCommand(armCommand);

        configureBindings();
    }

    private void configureBindings() {
        calibrateDriveButton.onTrue(new CalibrateSwerveDriveCommand(drive));
        calibrateTurnButton.whileTrue(new CalibrateSwerveTurnCommand(drive, this::getInput));
        coastSwerveButton.whileTrue(new CoastSwerveDriveCommand(drive));

        aimSpeakerTrigger.whileTrue(new AimSpeakerCommand(arm, drive, positioning));

        SysIdRoutine routine = arm.getShoulderRoutine(null);
        runSysidTrigger.whileTrue(Commands.defer(
                () -> {
                    switch (sysidMode.getSelected()) {
                        case QUASISTATIC_FORWARD:
                            return routine.quasistatic(SysIdRoutine.Direction.kForward);
                        case QUASISTATIC_REVERSE:
                            return routine.quasistatic(SysIdRoutine.Direction.kReverse);
                        case DYNAMIC_FORWARD:
                            return routine.dynamic(SysIdRoutine.Direction.kForward);
                        case DYNAMIC_REVERSE:
                            return routine.dynamic(SysIdRoutine.Direction.kReverse);
                        case NONE:
                        default:
                            return Commands.none();
                    }
                },
                Set.of(arm)));

        RobotModeTriggers.teleop()
                .and(() -> shouldPlayEnableTone.getBoolean(false))
                .whileTrue(startupOrchestraCommand);
    }

    private MoInput getInput() {
        return inputChooser.getSelected();
    }

    public Command getAutonomousCommand() {
        return PathPlannerCommands.getFollowPathCommand(
                drive, positioning, autoPathEntry.get(), autoAssumeAtStartEntry.get());
    }
}
