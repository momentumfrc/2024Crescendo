// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;

public class MoShuffleboard {
    private static MoShuffleboard instance;

    public static MoShuffleboard getInstance() {
        if (instance == null) {
            instance = new MoShuffleboard();
        }
        return instance;
    }

    private enum SysIdMode {
        NONE,
        QUASISTATIC_FORWARD,
        QUASISTATIC_REVERSE,
        DYNAMIC_FORWARD,
        DYNAMIC_REVERSE
    };

    private final SendableChooser<SysIdMode> sysidMode = MoShuffleboard.enumToChooser(SysIdMode.class);

    public final ShuffleboardTab driveTab;
    public final ShuffleboardTab armTab;
    public final ShuffleboardTab intakeTab;
    public final ShuffleboardTab shooterTab;
    public final ShuffleboardTab climberTab;
    public final ShuffleboardTab settingsTab;
    public final ShuffleboardTab autoTab;
    public final Field2d field;

    private final GenericEntry voltRampEntry;
    private final GenericEntry voltEntry;

    private MoShuffleboard() {
        settingsTab = Shuffleboard.getTab("Settings");
        autoTab = Shuffleboard.getTab("Auto");
        driveTab = Shuffleboard.getTab("Drive");
        armTab = Shuffleboard.getTab("Arm");
        intakeTab = Shuffleboard.getTab("Intake");
        climberTab = Shuffleboard.getTab("Climber");
        shooterTab = Shuffleboard.getTab("Shooter");

        field = new Field2d();
        driveTab.add(field).withSize(5, 3);

        settingsTab.add("Sysid Mode", sysidMode);

        var sysidGroup = settingsTab
                .getLayout("Sysid Settings", BuiltInLayouts.kList)
                .withSize(2, 1)
                .withProperties(Map.of("Label position", "RIGHT"));
        voltRampEntry = sysidGroup.add("Volts Ramp", 1.5).getEntry();
        voltEntry = sysidGroup.add("Volts Step", 2).getEntry();
    }

    public SysIdRoutine.Config getSysidConfig() {
        var voltsPerSec = Units.Volts.per(Units.Second);
        return new SysIdRoutine.Config(
                voltsPerSec.of(voltRampEntry.getDouble(1.5)),
                Units.Volts.of(voltEntry.getDouble(3)),
                Units.Seconds.of(45));
    }

    public Command getSysidCommand(Supplier<SysIdRoutine> routineSupplier, Subsystem... subsystems) {
        return Commands.defer(
                () -> {
                    var routine = routineSupplier.get();
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
                Set.of(subsystems));
    }

    public static <T extends Enum<?>> SendableChooser<T> enumToChooser(Class<T> toConvert) {
        boolean setDefault = true;
        var chooser = new SendableChooser<T>();
        for (T entry : toConvert.getEnumConstants()) {
            if (setDefault) {
                chooser.setDefaultOption(entry.name(), entry);
                setDefault = false;
            } else {
                chooser.addOption(entry.name(), entry);
            }
        }
        return chooser;
    }
}
