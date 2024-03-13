// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.component.Climber;
import frc.robot.util.MoPrefs;
import frc.robot.util.MoShuffleboard;

public class ClimbSubsystem extends SubsystemBase {
    public final Climber leftClimber;
    public final Climber rightClimber;

    public ClimbSubsystem() {
        super("Climber");

        var leftSparkMax = new CANSparkMax(Constants.CLIMBER_LEFT.address(), CANSparkMax.MotorType.kBrushless);
        var rightSparkMax = new CANSparkMax(Constants.CLIMBER_RIGHT.address(), CANSparkMax.MotorType.kBrushless);

        leftClimber =
                new Climber("Climber Left", leftSparkMax, MoPrefs.climberCurrentThreshold, MoPrefs.climberLimitTime);
        rightClimber =
                new Climber("Climber Right", rightSparkMax, MoPrefs.climberCurrentThreshold, MoPrefs.climberLimitTime);

        MoShuffleboard.getInstance().settingsTab.add("Invalidate Climber Zeros", new InstantCommand(() -> {
            leftClimber.invalidateZero();
            rightClimber.invalidateZero();
        }));
    }

    public void stop() {
        leftClimber.stop();
        rightClimber.stop();
    }
}
