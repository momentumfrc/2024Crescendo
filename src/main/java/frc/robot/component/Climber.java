// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.component;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import frc.robot.encoder.MoEncoder;
import frc.robot.util.MoPrefs;
import frc.robot.util.MoShuffleboard;

/* A single side climber */
public class Climber {
    public final CANSparkMax winch;
    public final MoEncoder<Distance> encoder;

    public final GenericEntry hasZero;

    public Climber(String name, CANSparkMax winch) {
        this.winch = winch;
        winch.restoreFactoryDefaults();

        this.encoder = MoEncoder.forSparkRelative(winch.getEncoder(), Units.Centimeters);

        MoPrefs.climberEncoderScale.subscribe(this.encoder::setConversionFactor, true);
        MoPrefs.climberZeroCurrentCutoff.subscribe(
                current -> winch.setSmartCurrentLimit((int) current.in(Units.Amps)), true);

        winch.setIdleMode(IdleMode.kBrake);

        winch.setSoftLimit(SoftLimitDirection.kReverse, 0);
        MoPrefs.climberMaximum.subscribe(
                max -> winch.setSoftLimit(
                        SoftLimitDirection.kForward, (float) max.in(this.encoder.getInternalEncoderUnits())),
                true);

        winch.enableSoftLimit(SoftLimitDirection.kReverse, true);
        winch.enableSoftLimit(SoftLimitDirection.kForward, true);

        var layout = MoShuffleboard.getInstance()
                .climberTab
                .getLayout(name, BuiltInLayouts.kList)
                .withSize(1, 2);

        hasZero = layout.add("Has Zero?", false)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .getEntry();

        layout.addDouble("Position (cm)", () -> winch.getEncoder().getPosition());
        layout.addDouble("Current (A)", () -> winch.getOutputCurrent());
    }

    public boolean hasZero() {
        return hasZero.getBoolean(false);
    }

    public void runWinch(double power) {
        this.winch.set(power);
    }

    public void zero(double power, Timer currentTimer) {
        Measure<Current> current = Units.Amps.of(this.winch.getOutputCurrent());

        if (current.gte(MoPrefs.climberZeroCurrentCutoff.get())) {
            if (currentTimer.hasElapsed(MoPrefs.climberZeroTimeCutoff.get().in(Units.Seconds))) {
                this.encoder.setPosition(Units.Centimeters.of(0));
                this.hasZero.setBoolean(true);
            }
        } else {
            currentTimer.restart();
        }

        this.runWinch(-Math.abs(MoPrefs.intakeZeroPwr.get()));
    }

    public void enableWinchSoftLimitReverse(boolean enable) {
        this.winch.enableSoftLimit(SoftLimitDirection.kReverse, enable);
    }

    public void stop() {
        this.winch.stopMotor();
    }
}
