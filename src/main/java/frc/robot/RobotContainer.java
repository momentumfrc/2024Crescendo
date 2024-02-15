// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.command.CalibrateSwerveDriveCommand;
import frc.robot.command.CalibrateSwerveTurnCommand;
import frc.robot.command.OrchestraCommand;
import frc.robot.command.TeleopDriveCommand;
import frc.robot.input.MoInput;
import frc.robot.input.SingleControllerInput;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.PositioningSubsystem;
import frc.robot.util.MoShuffleboard;

public class RobotContainer {
    private AHRS gyro = new AHRS(SerialPort.Port.kMXP);

    // Subsystems
    private DriveSubsystem drive = new DriveSubsystem(gyro);
    private PositioningSubsystem positioning = new PositioningSubsystem(gyro, drive);

    // Commands
    private TeleopDriveCommand driveCommand = new TeleopDriveCommand(drive, positioning, this::getInput);
    private OrchestraCommand startupOrchestraCommand = new OrchestraCommand(drive, this::getInput, "windows-xp.chrp");

    private SendableChooser<MoInput> inputChooser = new SendableChooser<>();

    private final NetworkButton calibrateDriveButton;
    private final NetworkButton calibrateTurnButton;

    private final GenericEntry shouldPlayEnableTone = MoShuffleboard.getInstance()
            .settingsTab
            .add("Tone on Enable", false)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();

    public RobotContainer() {
        inputChooser.setDefaultOption("Single Controller", new SingleControllerInput(Constants.DRIVE_F310));
        MoShuffleboard.getInstance().settingsTab.add("Controller Mode", inputChooser);

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

        drive.setDefaultCommand(driveCommand);

        configureBindings();
    }

    private void configureBindings() {
        calibrateDriveButton.onTrue(new CalibrateSwerveDriveCommand(drive));
        calibrateTurnButton.whileTrue(new CalibrateSwerveTurnCommand(drive, this::getInput));

        RobotModeTriggers.teleop()
                .and(() -> shouldPlayEnableTone.getBoolean(false))
                .whileTrue(startupOrchestraCommand);
    }

    private MoInput getInput() {
        return inputChooser.getSelected();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
