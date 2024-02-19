// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import java.util.EnumSet;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Function;

/** Robot preferences, accessible through Shuffleboard */
public class MoPrefs {
    public static final Pref<Double> maxDriveSpeed = unitlessDoublePref("Drive Max Speed", 6000.0);
    public static final Pref<Double> maxTurnSpeed = unitlessDoublePref("Turn Max Speed", 6000.0);
    public static final Pref<Double> driveDeadzone = unitlessDoublePref("Drive Deadzone", 0.05);
    public static final Pref<Double> driveCurve = unitlessDoublePref("Drive Curve", 1);
    public static Pref<Double> driveSlowSpeed = unitlessDoublePref("Drive Slow Speed", 0.5);
    public static Pref<Double> turnSlowSpeed = unitlessDoublePref("Turn Slow Speed", 0.25);
    public static Pref<Double> driveRampTime = unitlessDoublePref("Drive Ramp Time", 0.25);

    public static Pref<Double> flZero = unitlessDoublePref("FL Drive Zero", 0);
    public static Pref<Double> flScale = unitlessDoublePref("FL Drive Scale", 1);
    public static Pref<Double> frZero = unitlessDoublePref("FR Drive Zero", 0);
    public static Pref<Double> frScale = unitlessDoublePref("FR Drive Scale", 1);
    public static Pref<Double> rlZero = unitlessDoublePref("RL Drive Zero", 0);
    public static Pref<Double> rlScale = unitlessDoublePref("RL Drive Scale", 1);
    public static Pref<Double> rrZero = unitlessDoublePref("RR Drive Zero", 0);
    public static Pref<Double> rrScale = unitlessDoublePref("RR Drive Scale", 1);

    // Motor Scale: units of ticks per meter
    public static Pref<Double> flDriveMtrScale = unitlessDoublePref("FL Drive Motor Scale", 1);
    public static Pref<Double> frDriveMtrScale = unitlessDoublePref("FR Drive Motor Scale", 1);
    public static Pref<Double> rlDriveMtrScale = unitlessDoublePref("RL Drive Motor Scale", 1);
    public static Pref<Double> rrDriveMtrScale = unitlessDoublePref("RR Drive Motor Scale", 1);

    public final class UnitPref<U extends Unit<U>> {
        private final Pref<Double> basePref;
        private U storeUnits;

        public UnitPref(String key, U storeUnits, Measure<U> defaultValue) {
            this.basePref = MoPrefs.this
            .new Pref<>(
                    String.format("%s (%s)", key, storeUnits.symbol()),
                    defaultValue.in(storeUnits),
                    NetworkTableValue::getDouble,
                    NetworkTableEntry::setDouble);
        }

        public Measure<U> get() {
            return storeUnits.of(basePref.get());
        }

        public void set(Measure<U> value) {
            basePref.set(value.in(storeUnits));
        }

        public void subscribe(Consumer<Measure<U>> consumer) {
            subscribe(consumer, false);
        }

        public void subscribe(Consumer<Measure<U>> consumer, boolean notifyImmediately) {
            basePref.subscribe((value) -> consumer.accept(storeUnits.of(value)), notifyImmediately);
        }
    }

    public final class Pref<T> {
        public final String key;
        private Function<NetworkTableValue, T> getter;
        private BiFunction<NetworkTableEntry, T, Boolean> setter;

        private final NetworkTableEntry entry;

        private Consumer<T> subscriber = null;

        public Pref(
                String key,
                T defaultValue,
                Function<NetworkTableValue, T> getter,
                BiFunction<NetworkTableEntry, T, Boolean> setter) {
            if (key.contains("/")) {
                throw new IllegalArgumentException("Pref keys must not contain '/'");
            }

            this.key = key;
            this.getter = getter;
            this.setter = setter;

            this.entry = table.getEntry(key);
            this.entry.setDefaultValue(defaultValue);
            this.entry.setPersistent();
        }

        public T get() {
            return getter.apply(entry.getValue());
        }

        public void set(T value) {
            setter.apply(entry, value);
        }

        public void subscribe(Consumer<T> consumer) {
            subscribe(consumer, false);
        }

        public void subscribe(Consumer<T> consumer, boolean notifyImmediately) {
            if (subscriber != null) {
                subscriber = subscriber.andThen(consumer);
            } else {
                subscriber = consumer;
                entry.getInstance()
                        .addListener(
                                entry,
                                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                                (e) -> consumer.accept(getter.apply(e.valueData.value)));
            }

            if (notifyImmediately) {
                consumer.accept(this.get());
            }
        }
    }

    private static MoPrefs instance;
    private NetworkTable table;
    private StringPublisher typePublisher;

    private static MoPrefs getInstance() {
        if (instance == null) {
            instance = new MoPrefs();
        }
        return instance;
    }

    private MoPrefs() {
        table = NetworkTableInstance.getDefault().getTable("Preferences");
        typePublisher = table.getStringTopic(".type").publish();
        typePublisher.set("RobotPreferences");
    }

    private static Pref<Double> unitlessDoublePref(String key, double defaultValue) {
        return getInstance().new Pref<>(key, defaultValue, NetworkTableValue::getDouble, NetworkTableEntry::setDouble);
    }

    private static UnitPref<Angle> rotationsPref(String key, Measure<Angle> defaultValue) {
        return getInstance().new UnitPref<>(key, Units.Rotations, defaultValue);
    }

    private static UnitPref<Angle> degreesPref(String key, Measure<Angle> defaultValue) {
        return getInstance().new UnitPref<>(key, Units.Degrees, defaultValue);
    }

    private static UnitPref<Distance> metersPref(String key, Measure<Distance> defaultValue) {
        return getInstance().new UnitPref<>(key, Units.Meters, defaultValue);
    }

    private static UnitPref<Velocity<Distance>> metersPerSecPref(String key, Measure<Velocity<Distance>> defaultValue) {
        return getInstance().new UnitPref<>(key, Units.MetersPerSecond, defaultValue);
    }

    private static UnitPref<Velocity<Angle>> rotationsPerSecPref(String key, Measure<Velocity<Angle>> defaultValue) {
        return getInstance().new UnitPref<>(key, Units.RotationsPerSecond, defaultValue);
    }
}
