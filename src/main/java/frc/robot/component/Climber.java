// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.component;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import frc.robot.util.MoPrefs;
import frc.robot.util.MoPrefs.UnitPref;
import frc.robot.util.MoShuffleboard;
import frc.robot.util.MoUnits;

/* A single side climber */
public class Climber {
    public final CANSparkMax winch;
    public final SparkPIDController winchPID;
    public final CurrentTrigger zeroLimit;
    private boolean hasZero = false;

    public Climber(
            String name, CANSparkMax winch, UnitPref<Current> zeroCurrentLimit, UnitPref<Time> zeroTriggerDuration) {
        this.winch = winch;
        this.winchPID = winch.getPIDController();
        this.zeroLimit = CurrentTrigger.ofSparkMax(winch, zeroCurrentLimit, zeroTriggerDuration);

        var layout = MoShuffleboard.getInstance()
                .matchTab
                .getLayout(name, BuiltInLayouts.kList)
                .withSize(1, 2);
        layout.addBoolean("At Zero Limit", this::atZeroLimit);
        layout.addBoolean("Has Zero", this::hasZero);
        layout.addDouble("Encoder Pos", () -> this.winch.getEncoder().getPosition());
        layout.addDouble("Current", () -> this.winch.getOutputCurrent());
    }

    public void invalidateZero() {
        hasZero = false;
    }

    public boolean hasZero() {
        return hasZero;
    }

    private void runWinch(double power) {
        if (MoPrefs.climberPid.get()) {
            winchPID.setReference(
                    power * MoPrefs.climberMotorSpeed.get().in(MoUnits.RotationsPerMinute),
                    CANSparkMax.ControlType.kVelocity);
        } else {
            winchPID.setReference(power, CANSparkMax.ControlType.kDutyCycle);
        }
    }

    public boolean atZeroLimit() {
        return zeroLimit.getAsBoolean();
    }

    public void raise(double power) {
        if (!hasZero) {
            this.runWinch(power);

            return;
        }

        if (power > 0 && winch.getEncoder().getPosition() >= MoPrefs.climberZeroThreshold.get()) {
            this.runWinch(0);
            return;
        }

        if (power < 0 && winch.getEncoder().getPosition() <= MoPrefs.climberZeroThreshold.get()) {
            this.runWinch(0);
            return;
        }

        this.runWinch(power);
    }

    public void zero(double power) {
        if (!hasZero) {
            winch.set(-power);

            if (atZeroLimit()) {
                winch.getEncoder().setPosition(0);
                hasZero = true;
                winch.set(0);
            }
        } else {
            winch.set(0);
        }
    }

    public void stop() {
        winch.stopMotor();
    }
}
