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
import java.util.EnumSet;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Function;

/** Robot preferences, accessible through Shuffleboard */
public class MoPrefs {
    public static final Pref<Double> maxDriveSpeed = doublePref("Drive Max Speed m/s", 6000.0);
    public static final Pref<Double> maxTurnSpeed = doublePref("Turn Max Speed m/s", 6000.0);
    public static final Pref<Double> driveDeadzone = doublePref("Drive Deadzone", 0.05);
    public static final Pref<Double> driveCurve = doublePref("Drive Curve", 1);
    public static Pref<Double> driveSlowSpeed = doublePref("Drive Slow Speed", 0.5);
    public static Pref<Double> turnSlowSpeed = doublePref("Turn Slow Speed", 0.25);
    public static Pref<Double> driveRampTime = doublePref("Drive Ramp Time", 0.25);

    public static Pref<Double> flZero = doublePref("FL Drive Zero", 0);
    public static Pref<Double> flScale = doublePref("FL Drive Scale", 1);
    public static Pref<Double> frZero = doublePref("FR Drive Zero", 0);
    public static Pref<Double> frScale = doublePref("FR Drive Scale", 1);
    public static Pref<Double> rlZero = doublePref("RL Drive Zero", 0);
    public static Pref<Double> rlScale = doublePref("RL Drive Scale", 1);
    public static Pref<Double> rrZero = doublePref("RR Drive Zero", 0);
    public static Pref<Double> rrScale = doublePref("RR Drive Scale", 1);

    // Motor Scale: units of ticks per meter
    public static Pref<Double> flDriveMtrScale = doublePref("FL Drive Motor Scale", 1);
    public static Pref<Double> frDriveMtrScale = doublePref("FR Drive Motor Scale", 1);
    public static Pref<Double> rlDriveMtrScale = doublePref("RL Drive Motor Scale", 1);
    public static Pref<Double> rrDriveMtrScale = doublePref("RR Drive Motor Scale", 1);

    public static Pref<Double> chassisSizeX = doublePref("Chassis Size X", 1);
    public static Pref<Double> chassisSizeY = doublePref("Chassis Size Y", 1);

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

    private static Pref<Double> doublePref(String key, double defaultValue) {
        return getInstance()
        .new Pref<>(key, defaultValue, NetworkTableValue::getDouble, NetworkTableEntry::setDouble);
    }
}
