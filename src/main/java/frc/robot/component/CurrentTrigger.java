// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.component;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.MoPrefs.UnitPref;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/** Detects overcurrent on motor controllers, serves as limit detection */
public class CurrentTrigger implements BooleanSupplier {
    private final Supplier<Measure<Current>> current;
    public final Trigger trigger;
    public Measure<Current> currentThreshold;
    public Measure<Time> triggerDuration;

    private long triggerStartTimeMs = 0;

    public CurrentTrigger(
            Supplier<Measure<Current>> currentGetter,
            Measure<Current> currentThreshold,
            Measure<Time> triggerDuration) {
        this.current = currentGetter;
        this.currentThreshold = currentThreshold;
        this.triggerDuration = triggerDuration;

        this.trigger = new Trigger(this);
    }

    public static CurrentTrigger ofSparkMax(
            CANSparkMax spark, UnitPref<Current> limitPref, UnitPref<Time> durationPref) {
        var trigger =
                new CurrentTrigger(() -> Units.Amps.of(spark.getOutputCurrent()), limitPref.get(), durationPref.get());

        limitPref.subscribe(limit -> trigger.currentThreshold = limit);
        durationPref.subscribe(dur -> trigger.triggerDuration = dur);

        return trigger;
    }

    @Override
    public boolean getAsBoolean() {
        if (current.get().gt(currentThreshold)) {
            return (System.currentTimeMillis() - triggerStartTimeMs) > triggerDuration.in(Units.Milliseconds);
        } else {
            triggerStartTimeMs = System.currentTimeMillis();
            return false;
        }
    }
}
