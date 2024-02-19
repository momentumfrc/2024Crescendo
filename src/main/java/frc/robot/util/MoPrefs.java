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
import edu.wpi.first.units.Per;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import java.util.EnumSet;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Function;

/** Robot preferences, accessible through Shuffleboard */
public class MoPrefs {
    public static final UnitPref<Velocity<Distance>> maxDriveSpeed =
            metersPerSecPref("Drive Max Speed", Units.MetersPerSecond.of(6));
    public static final UnitPref<Velocity<Angle>> maxTurnSpeed =
            rotationsPerSecPref("Turn Max Speed", Units.RotationsPerSecond.of(3));
    public static final Pref<Double> driveDeadzone = unitlessDoublePref("Drive Deadzone", 0.05);
    public static final Pref<Double> driveCurve = unitlessDoublePref("Drive Curve", 1);
    public static Pref<Double> driveSlowSpeed = unitlessDoublePref("Drive Slow Speed", 0.5);
    public static Pref<Double> turnSlowSpeed = unitlessDoublePref("Turn Slow Speed", 0.25);
    public static Pref<Double> driveRampTime = unitlessDoublePref("Drive Ramp Time", 0.25);

    public static UnitPref<Angle> flZero = rotationsPref("FL Drive Zero", Units.Rotations.of(0));
    public static UnitPref<Per<MoUnits.EncoderAngle, Angle>> flRotScale =
            encoderTicksPerRotationPref("FL Drive Rot Scale", MoUnits.EncoderTicksPerRotation.of(0.5));
    public static UnitPref<Angle> frZero = rotationsPref("FR Drive Zero", Units.Rotations.of(0));
    public static UnitPref<Per<MoUnits.EncoderAngle, Angle>> frRotScale =
            encoderTicksPerRotationPref("FR Drive Rot Scale", MoUnits.EncoderTicksPerRotation.of(0.5));
    public static UnitPref<Angle> rlZero = rotationsPref("RL Drive Zero", Units.Rotations.of(0));
    public static UnitPref<Per<MoUnits.EncoderAngle, Angle>> rlRotScale =
            encoderTicksPerRotationPref("RL Drive Rot Scale", MoUnits.EncoderTicksPerRotation.of(0.5));
    public static UnitPref<Angle> rrZero = rotationsPref("RR Drive Zero", Units.Rotations.of(0));
    public static UnitPref<Per<MoUnits.EncoderAngle, Angle>> rrRotScale =
            encoderTicksPerRotationPref("RR Drive Rot Scale", MoUnits.EncoderTicksPerRotation.of(0.5));

    public static UnitPref<Per<MoUnits.EncoderAngle, Distance>> flDistScale =
            encoderTicksPerMeterPref("FL Drive Dist Scale", MoUnits.EncoderTicksPerMeter.of(1));
    public static UnitPref<Per<MoUnits.EncoderAngle, Distance>> frDistScale =
            encoderTicksPerMeterPref("FR Drive Dist Scale", MoUnits.EncoderTicksPerMeter.of(1));
    public static UnitPref<Per<MoUnits.EncoderAngle, Distance>> rlDistScale =
            encoderTicksPerMeterPref("RL Drive Dist Scale", MoUnits.EncoderTicksPerMeter.of(1));
    public static UnitPref<Per<MoUnits.EncoderAngle, Distance>> rrDistScale =
            encoderTicksPerMeterPref("RR Drive Dist Scale", MoUnits.EncoderTicksPerMeter.of(1));

    public final class UnitPref<U extends Unit<U>> {
        private final Pref<Double> basePref;
        private final U storeUnits;

        public UnitPref(String key, U storeUnits, Measure<U> defaultValue) {
            String symbol = storeUnits.symbol().replaceAll("/", "_");

            this.basePref = MoPrefs.this
            .new Pref<>(
                    String.format("%s (%s)", key, symbol),
                    defaultValue.in(storeUnits),
                    NetworkTableValue::getDouble,
                    NetworkTableEntry::setDouble);

            this.storeUnits = storeUnits;
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

    private static UnitPref<Per<MoUnits.EncoderAngle, Distance>> encoderTicksPerMeterPref(
            String key, Measure<Per<MoUnits.EncoderAngle, Distance>> defaultValue) {
        return getInstance().new UnitPref<>(key, MoUnits.EncoderTicksPerMeter, defaultValue);
    }

    private static UnitPref<Per<MoUnits.EncoderAngle, Angle>> encoderTicksPerRotationPref(
            String key, Measure<Per<MoUnits.EncoderAngle, Angle>> defaultValue) {
        return getInstance().new UnitPref<>(key, MoUnits.EncoderTicksPerRotation, defaultValue);
    }
}
