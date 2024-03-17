// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.cameraserver.CameraServer;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.command.HandoffCommand;
import frc.robot.command.IntakeSourceCommand;
import frc.robot.command.OrchestraCommand;
import frc.robot.command.TeleopDriveCommand;
import frc.robot.command.arm.TeleopArmCommand;
import frc.robot.command.arm.WaitForArmSetpointCommand;
import frc.robot.command.calibration.CalibrateSwerveDriveCommand;
import frc.robot.command.calibration.CalibrateSwerveTurnCommand;
import frc.robot.command.calibration.CoastSwerveDriveCommand;
import frc.robot.command.climb.TeleopClimbCommand;
import frc.robot.command.climb.ZeroClimbersCommand;
import frc.robot.command.intake.TeleopIntakeCommand;
import frc.robot.command.intake.ZeroIntakeCommand;
import frc.robot.command.shooter.BackoffShooterCommand;
import frc.robot.command.shooter.IdleShooterCommand;
import frc.robot.command.shooter.ShootAmpCommand;
import frc.robot.command.shooter.ShootSpeakerCommand;
import frc.robot.command.shooter.SpinupShooterCommand;
import frc.robot.component.ArmSetpointManager.ArmSetpoint;
import frc.robot.input.DualControllerInput;
import frc.robot.input.JoystickDualControllerInput;
import frc.robot.input.MoInput;
import frc.robot.input.SingleControllerInput;
import frc.robot.subsystem.ArmSubsystem;
import frc.robot.subsystem.AutoBuilderSubsystem;
import frc.robot.subsystem.ClimbSubsystem;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.IntakeSubsystem;
import frc.robot.subsystem.PositioningSubsystem;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.util.MoPrefs;
import frc.robot.util.MoShuffleboard;

public class RobotContainer {
    private AHRS gyro = new AHRS(SerialPort.Port.kMXP);

    // Subsystems
    private DriveSubsystem drive = new DriveSubsystem(gyro);
    private PositioningSubsystem positioning = new PositioningSubsystem(gyro, drive);
    private ArmSubsystem arm = new ArmSubsystem();
    private ShooterSubsystem shooter = new ShooterSubsystem();
    private IntakeSubsystem intake = new IntakeSubsystem();
    private ClimbSubsystem climb = new ClimbSubsystem();
    private AutoBuilderSubsystem autoBuilder = new AutoBuilderSubsystem(positioning, arm, shooter);

    // Commands
    private TeleopDriveCommand driveCommand = new TeleopDriveCommand(drive, positioning, this::getInput);
    private TeleopArmCommand armCommand = new TeleopArmCommand(arm, this::getInput);
    private TeleopClimbCommand climbCommand = new TeleopClimbCommand(climb, this::getInput);
    private TeleopIntakeCommand intakeCommand = new TeleopIntakeCommand(intake, this::getInput);
    private IdleShooterCommand idleShooterCommand = new IdleShooterCommand(shooter, this::getInput);
    private Command handoffCommand =
            new WaitForArmSetpointCommand(arm, ArmSetpoint.HANDOFF).andThen(new HandoffCommand(intake, shooter));

    private Command shootSpeakerCommand = new WaitForArmSetpointCommand(arm, ArmSetpoint.SPEAKER)
            .deadlineWith(new SpinupShooterCommand(shooter, () -> MoPrefs.flywheelSpeakerSetpoint.get()))
            .andThen(new ShootSpeakerCommand(shooter))
            .withName("ShootSpeakerCommand");

    private Command shootAmpCommand = new WaitForArmSetpointCommand(arm, ArmSetpoint.AMP)
            .andThen(new ShootAmpCommand(shooter))
            .withName("ShootAmpCommand");

    private OrchestraCommand startupOrchestraCommand = new OrchestraCommand(drive, this::getInput, "windows-xp.chrp");

    private Command backoffShooterCommand = new BackoffShooterCommand(shooter);

    private ZeroIntakeCommand reZeroIntake = new ZeroIntakeCommand(intake);
    private ZeroClimbersCommand reZeroClimbers = new ZeroClimbersCommand(climb);

    private SendableChooser<MoInput> inputChooser = new SendableChooser<>();

    private final NetworkButton calibrateDriveButton;
    private final NetworkButton calibrateTurnButton;
    private final NetworkButton coastSwerveButton;
    private final NetworkButton burnFlashButton;

    private final Trigger runSysidTrigger;
    private final Trigger shootSpeakerTrigger;
    private final Trigger shootAmpTrigger;
    private final Trigger reZeroIntakeTrigger;
    private final Trigger reZeroClimbTrigger;
    private final Trigger handoffTrigger;
    private final Trigger intakeSourceTrigger;

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

        BooleanEntry burnFlashEntry = NetworkTableInstance.getDefault()
                .getTable("Settings")
                .getBooleanTopic("Burn Flash")
                .getEntry(false);
        burnFlashEntry.setDefault(false);
        burnFlashButton = new NetworkButton(burnFlashEntry);

        runSysidTrigger = new Trigger(() -> getInput().getRunSysId());
        shootSpeakerTrigger = new Trigger(() -> getInput().getShootTargetDebounced() == MoInput.ShootTarget.SPEAKER);
        shootAmpTrigger = new Trigger(() -> getInput().getShootTargetDebounced() == MoInput.ShootTarget.AMP);
        reZeroClimbTrigger = new Trigger(() -> !climb.bothZeroed());
        reZeroIntakeTrigger = new Trigger(() -> !intake.isDeployZeroed.getBoolean(false));
        handoffTrigger = new Trigger(() -> getInput().getHandoff());
        intakeSourceTrigger =
                new Trigger(() -> getInput().getArmSetpoint().orElse(ArmSetpoint.STOW) == ArmSetpoint.SOURCE);

        drive.setDefaultCommand(driveCommand);
        arm.setDefaultCommand(armCommand);
        intake.setDefaultCommand(intakeCommand);
        shooter.setDefaultCommand(idleShooterCommand);
        climb.setDefaultCommand(climbCommand);

        CameraServer.startAutomaticCapture();

        configureBindings();
    }

    private void configureBindings() {
        calibrateDriveButton.onTrue(new CalibrateSwerveDriveCommand(drive));
        calibrateTurnButton.whileTrue(new CalibrateSwerveTurnCommand(drive, this::getInput));
        coastSwerveButton.whileTrue(new CoastSwerveDriveCommand(drive));
        burnFlashButton.onTrue(Commands.runOnce(() -> {
            shooter.burnFlash();
        }));

        var tuneSetpointSubscriber = MoShuffleboard.getInstance().tuneSetpointSubscriber;
        shootSpeakerTrigger.whileTrue(shootSpeakerCommand);
        shootAmpTrigger.whileTrue(shootAmpCommand);

        reZeroIntakeTrigger.onTrue(reZeroIntake);

        handoffTrigger.and(() -> !tuneSetpointSubscriber.getBoolean(false)).whileTrue(handoffCommand);
        handoffTrigger.onFalse(backoffShooterCommand);

        intakeSourceTrigger.whileTrue(new IntakeSourceCommand(shooter));
        intakeSourceTrigger.onFalse(backoffShooterCommand);

        reZeroClimbTrigger.onTrue(reZeroClimbers);

        runSysidTrigger.whileTrue(Commands.print("STARTING SYSID...")
                .andThen(MoShuffleboard.getInstance().getSysidCommand(intake::getDeployRoutine, intake)));

        RobotModeTriggers.autonomous()
                .or(RobotModeTriggers.teleop())
                .and(reZeroIntakeTrigger)
                .onTrue(reZeroIntake);

        RobotModeTriggers.autonomous()
                .or(RobotModeTriggers.teleop())
                .and(reZeroClimbTrigger)
                .onTrue(reZeroClimbers);

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
