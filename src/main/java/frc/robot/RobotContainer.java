// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.command.CompositeCommands;
import frc.robot.command.OrchestraCommand;
import frc.robot.command.TeleopDriveCommand;
import frc.robot.command.arm.TeleopArmCommand;
import frc.robot.command.calibration.CalibrateSwerveDriveCommand;
import frc.robot.command.calibration.CalibrateSwerveTurnCommand;
import frc.robot.command.calibration.CoastSwerveDriveCommand;
import frc.robot.command.intake.TeleopIntakeCommand;
import frc.robot.command.intake.ZeroIntakeCommand;
import frc.robot.command.shooter.IdleShooterCommand;
import frc.robot.input.DualControllerInput;
import frc.robot.input.JoystickDualControllerInput;
import frc.robot.input.MoInput;
import frc.robot.input.SingleControllerInput;
import frc.robot.subsystem.ArmSubsystem;
import frc.robot.subsystem.AutoBuilderSubsystem;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.IntakeSubsystem;
import frc.robot.subsystem.PositioningSubsystem;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.util.MoShuffleboard;
import java.util.Set;

public class RobotContainer {
    private AHRS gyro = new AHRS(SerialPort.Port.kMXP);

    // Subsystems
    private DriveSubsystem drive = new DriveSubsystem(gyro);
    private PositioningSubsystem positioning = new PositioningSubsystem(gyro, drive);
    private ArmSubsystem arm = new ArmSubsystem();
    private ShooterSubsystem shooter = new ShooterSubsystem();
    private IntakeSubsystem intake = new IntakeSubsystem();
    private AutoBuilderSubsystem autoBuilder = new AutoBuilderSubsystem(positioning);

    // Commands
    private TeleopDriveCommand driveCommand = new TeleopDriveCommand(drive, positioning, this::getInput);
    private TeleopArmCommand armCommand = new TeleopArmCommand(arm, this::getInput);
    private TeleopIntakeCommand intakeCommand = new TeleopIntakeCommand(intake, this::getInput);
    private IdleShooterCommand idleShooterCommand = new IdleShooterCommand(shooter);
    private OrchestraCommand startupOrchestraCommand = new OrchestraCommand(drive, this::getInput, "windows-xp.chrp");

    private ZeroIntakeCommand rezeroIntake = new ZeroIntakeCommand(intake);

    private SendableChooser<MoInput> inputChooser = new SendableChooser<>();

    private final NetworkButton calibrateDriveButton;
    private final NetworkButton calibrateTurnButton;
    private final NetworkButton coastSwerveButton;

    private final GenericSubscriber tuneSetpointSubscriber;
    private final GenericSubscriber tuneShooterAngleSubscriber;

    private final Trigger runSysidTrigger;
    private final Trigger shootSpeakerTrigger;
    private final Trigger shootAmpTrigger;
    private final Trigger rezeroIntakeTrigger;

    private final GenericEntry shouldPlayEnableTone = MoShuffleboard.getInstance()
            .settingsTab
            .add("Tone on Enable", false)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();

    public RobotContainer() {
        inputChooser.setDefaultOption(
                "Joystick Drive, F310 Arm", new JoystickDualControllerInput(Constants.JOYSTICK, Constants.ARM_F310));
        inputChooser.addOption("Dual Controller", new DualControllerInput(Constants.DRIVE_F310, Constants.ARM_F310));
        inputChooser.addOption("Single Controller", new SingleControllerInput(Constants.DRIVE_F310));
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

        BooleanEntry coastSwerveEntry = NetworkTableInstance.getDefault()
                .getTable("Settings")
                .getBooleanTopic("Coast Swerve Drive")
                .getEntry(false);
        coastSwerveEntry.setDefault(false);
        coastSwerveButton = new NetworkButton(coastSwerveEntry);

        tuneSetpointSubscriber = MoShuffleboard.getInstance()
                .settingsTab
                .add("Tune Arm Setpoints?", false)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .getEntry();
        tuneShooterAngleSubscriber = MoShuffleboard.getInstance()
                .settingsTab
                .add("Tune shooter angle?", false)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .getEntry();

        runSysidTrigger = new Trigger(() -> getInput().getRunSysId());
        shootSpeakerTrigger = new Trigger(() -> getInput().getShouldShootSpeaker());
        shootAmpTrigger = new Trigger(() -> getInput().getShouldShootAmp());
        rezeroIntakeTrigger = new Trigger(() -> !intake.isDeployZeroed.getBoolean(false));

        drive.setDefaultCommand(driveCommand);
        arm.setDefaultCommand(armCommand);
        intake.setDefaultCommand(intakeCommand);
        shooter.setDefaultCommand(idleShooterCommand);

        configureBindings();
    }

    private void configureBindings() {
        calibrateDriveButton.onTrue(new CalibrateSwerveDriveCommand(drive));
        calibrateTurnButton.whileTrue(new CalibrateSwerveTurnCommand(drive, this::getInput));
        coastSwerveButton.whileTrue(new CoastSwerveDriveCommand(drive));

        // Need to use deferred commands since the setpoints are passed in as constructor parameters but they might
        // change during operation. So we use DeferredCommand to only construct the command using the latest MoPrefs
        // right before we're about to execute the command.
        shootSpeakerTrigger
                .and(() -> !tuneSetpointSubscriber.getBoolean(false))
                .whileTrue(Commands.either(
                        CompositeCommands.tuneShootSpeakerCommand(drive, this::getInput, arm, shooter, positioning),
                        Commands.defer(
                                () -> CompositeCommands.shootSpeakerCommand(
                                        arm, drive, shooter, positioning, this::getInput),
                                Set.of(arm, drive, shooter)),
                        () -> tuneShooterAngleSubscriber.getBoolean(false)));

        shootAmpTrigger
                .and(() -> !tuneSetpointSubscriber.getBoolean(false))
                .whileTrue(Commands.defer(
                        () -> CompositeCommands.shootAmpCommand(arm, shooter, positioning),
                        Set.of(arm, drive, shooter)));

        rezeroIntakeTrigger.onTrue(rezeroIntake);

        runSysidTrigger.whileTrue(Commands.print("STARTING SYSID...")
                .andThen(MoShuffleboard.getInstance().getSysidCommand(shooter::getFlywheelUpperRoutine, shooter)));

        (RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop()))
                .and(() -> intake.isDeployZeroed.getBoolean(false))
                .onTrue(rezeroIntake);

        RobotModeTriggers.teleop()
                .and(() -> shouldPlayEnableTone.getBoolean(false))
                .whileTrue(startupOrchestraCommand);
    }

    private MoInput getInput() {
        return inputChooser.getSelected();
    }

    public Command getAutonomousCommand() {
        return autoBuilder.getAutonomousCommand(drive);
    }
}
