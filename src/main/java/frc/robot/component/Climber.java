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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import frc.robot.encoder.MoEncoder;
import frc.robot.util.MoPrefs;
import frc.robot.util.MoShuffleboard;

/* A single side climber */
public class Climber {
    private static final Measure<Current> CLIMBER_CURRENT_LIMIT = Units.Amps.of(25);

    public final CANSparkMax winch;
    public final MoEncoder<Distance> encoder;

    public final GenericEntry hasZero;

    public Climber(String name, CANSparkMax winch, boolean inverted) {
        this.winch = winch;
        winch.restoreFactoryDefaults();

        winch.setInverted(inverted);

        this.encoder = MoEncoder.forSparkRelative(winch.getEncoder(), Units.Centimeters);

        MoPrefs.climberEncoderScale.subscribe(this.encoder::setConversionFactor, true);
        winch.setSmartCurrentLimit((int) CLIMBER_CURRENT_LIMIT.in(Units.Amps));

        winch.setIdleMode(IdleMode.kBrake);

        winch.setSoftLimit(SoftLimitDirection.kReverse, 0);
        MoPrefs.climbMaxHeight.subscribe(
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

        layout.addDouble("Position (cm)", () -> encoder.getPosition().in(Units.Centimeters));
        layout.addDouble("Current (A)", () -> winch.getOutputCurrent());
    }

    public boolean hasZero() {
        return hasZero.getBoolean(false);
    }

    public void invalidateZero() {
        hasZero.setBoolean(false);
    }

    public void setZero(Measure<Distance> pos) {
        hasZero.setBoolean(true);
        this.encoder.setPosition(pos);
    }

    public void runWinch(double power) {
        this.winch.set(power);
    }

    public void enableWinchSoftLimitReverse(boolean enable) {
        this.winch.enableSoftLimit(SoftLimitDirection.kReverse, enable);
    }

    public void stop() {
        this.winch.stopMotor();
    }
}
