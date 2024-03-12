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
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Per;
import edu.wpi.first.units.Time;
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

    public static UnitPref<Velocity<Distance>> autoMaxModuleSpeed =
            metersPerSecPref("Auto Max Module Speed", Units.MetersPerSecond.of(4.5));
    public static final UnitPref<Per<MoUnits.EncoderAngle, Angle>> shoulderEncoderScale =
            encoderTicksPerRotationPref("Shoulder Encoder Scale", MoUnits.EncoderTicksPerRotation.of(144));
    public static final UnitPref<Per<MoUnits.EncoderAngle, Angle>> shoulderAbsEncoderScale =
            encoderTicksPerRotationPref("Shoulder Absolute Encoder Scale", MoUnits.EncoderTicksPerRotation.of(2.4));
    public static final UnitPref<Per<MoUnits.EncoderAngle, Angle>> wristEncoderScale =
            encoderTicksPerRotationPref("Wrist Encoder Scale", MoUnits.EncoderTicksPerRotation.of(1));

    public static final UnitPref<Angle> shoulderAbsZero =
            rotationsPref("Shoulder Absolute Zero", Units.Rotations.of(0));
    public static final UnitPref<Angle> wristAbsZero = rotationsPref("Wrist Absolute Zero", Units.Rotations.of(0));

    public static final UnitPref<Angle> shoulderMaxExtension =
            rotationsPref("Shoulder Max Rotations", Units.Degrees.of(165.0));
    public static final UnitPref<Angle> wristMaxExtension = rotationsPref("Wrist Max Rotations", Units.Rotations.of(1));

    public static final UnitPref<Angle> shoulderHorizontal =
            rotationsPref("Shoulder Horizontal", Units.Rotations.of(0.25));

    public static final UnitPref<Angle> wristZeroOffsetFromShoulder =
            rotationsPref("Wrist Offset From Shoulder", Units.Rotations.of(0.15));

    public static final UnitPref<Velocity<Angle>> shoulderMaxRps =
            rotationsPerSecPref("Shoulder Max Spd", Units.RotationsPerSecond.of(0.5));
    public static final UnitPref<Velocity<Angle>> wristMaxRps =
            rotationsPerSecPref("Wrist Max Speed", Units.RotationsPerSecond.of(0.5));

    public static final UnitPref<Per<MoUnits.EncoderAngle, Distance>> shooterRollerScale =
            encoderTicksPerCentimeterPref("Shooter Roller Scale", MoUnits.EncoderTicksPerCentimeter.of(1));

    public static final UnitPref<Per<MoUnits.EncoderAngle, Distance>> shooterFlywheelScale =
            encoderTicksPerCentimeterPref(
                    "Shooter Flywheel Scale", MoUnits.EncoderTicksPerCentimeter.of(0.04699456981059901));

    public static final UnitPref<Dimensionless> pidSetpointVarianceThreshold = getInstance()
    .new UnitPref<Dimensionless>("Shooter Setpoint Variance Threshold", Units.Percent, Units.Percent.of(5));

    public static final UnitPref<Velocity<Distance>> flywheelSpeakerSetpoint =
            centimetersPerSecPref("Speaker Flywheel Setpoint", MoUnits.CentimetersPerSec.of(10));

    public static final UnitPref<Velocity<Distance>> flywheelAmpSetpoint =
            centimetersPerSecPref("Amp Flywheel Setpoint", MoUnits.CentimetersPerSec.of(2));

    public static final UnitPref<Time> shooterRollerRunTime =
            secondsPref("Shooter Roller RunTime", Units.Seconds.of(2));

    public static final Pref<Double> armRampTime = unitlessDoublePref("Arm Ramp Time", 0.15);

    public static final UnitPref<Per<MoUnits.EncoderAngle, Distance>> intakeRollerScale =
            encoderTicksPerCentimeterPref("Intake Roller Scale", MoUnits.EncoderTicksPerCentimeter.of(1));
    public static final UnitPref<Per<MoUnits.EncoderAngle, Angle>> intakeDeployScale =
            encoderTicksPerRotationPref("Intake Deploy Scale", MoUnits.EncoderTicksPerRotation.of(1));
    public static final UnitPref<Angle> intakeDeployMaxExtension =
            rotationsPref("Intake Deploy Max Extension", Units.Rotations.of(0.5));

    public static final UnitPref<Velocity<Angle>> intakeDeployMaxSpeed =
            rotationsPerSecPref("Intake Deploy Max Speed", Units.RotationsPerSecond.of(0.25));

    public static final UnitPref<Time> intakeCurrentSenseTime =
            secondsPref("Intake Current Sense Time", Units.Seconds.of(0.1));
    public static final UnitPref<Current> intakeCurrentSenseThreshold =
            amperesPref("Intake Current Sense Threshold", Units.Amps.of(10));

    public static final UnitPref<Velocity<Distance>> intakeRollerSpeed =
            centimetersPerSecPref("Intake Roller Speed", MoUnits.CentimetersPerSec.of(1));

    public static final Pref<Double> intakeZeroPwr = unitlessDoublePref("Intake Zero Power", 0.2);
    public static final UnitPref<Current> intakeZeroCurrentCutoff =
            amperesPref("Intake Zero Current", Units.Amps.of(10));
    public static final UnitPref<Time> intakeZeroTimeCutoff = secondsPref("Intake Zero Time", Units.Seconds.of(0.1));
    public static final UnitPref<Angle> intakeZeroPosition =
            rotationsPref("Intake Zero Pos.", Units.Rotations.of(-0.05));

    public final class UnitPref<U extends Unit<U>> {
        private final Pref<Double> basePref;
        private final U storeUnits;

        // Keep a reference to the frequently used Measure instance, so that we're not constantly creating/destroying
        // instances. This reduces the amount of garbage collection that is needed. See:
        // https://docs.wpilib.org/en/stable/docs/software/basic-programming/java-units.html#mutability-and-object-creation
        private final MutableMeasure<U> currValue;

        public UnitPref(String key, U storeUnits, Measure<U> defaultValue) {
            String symbol = storeUnits.symbol().replaceAll("/", "_");

            currValue = MutableMeasure.mutable(defaultValue);

            this.basePref = MoPrefs.this
            .new Pref<>(
                    String.format("%s (%s)", key, symbol),
                    defaultValue.in(storeUnits),
                    NetworkTableValue::getDouble,
                    NetworkTableEntry::setDouble);

            this.storeUnits = storeUnits;
        }

        public Measure<U> get() {
            return currValue.mut_replace(basePref.get(), storeUnits);
        }

        public void set(Measure<U> value) {
            basePref.set(value.in(storeUnits));
        }

        public void subscribe(Consumer<Measure<U>> consumer) {
            subscribe(consumer, false);
        }

        public void subscribe(Consumer<Measure<U>> consumer, boolean notifyImmediately) {
            basePref.subscribe((value) -> consumer.accept(currValue.mut_replace(value, storeUnits)), notifyImmediately);
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

    private static UnitPref<Distance> centimetersPref(String key, Measure<Distance> defaultValue) {
        return getInstance().new UnitPref<>(key, Units.Centimeters, defaultValue);
    }

    private static UnitPref<Velocity<Distance>> centimetersPerSecPref(
            String key, Measure<Velocity<Distance>> defaultValue) {
        return getInstance().new UnitPref<>(key, MoUnits.CentimetersPerSec, defaultValue);
    }

    private static UnitPref<Velocity<Distance>> metersPerSecPref(String key, Measure<Velocity<Distance>> defaultValue) {
        return getInstance().new UnitPref<>(key, Units.MetersPerSecond, defaultValue);
    }

    private static UnitPref<Velocity<Angle>> rotationsPerSecPref(String key, Measure<Velocity<Angle>> defaultValue) {
        return getInstance().new UnitPref<>(key, Units.RotationsPerSecond, defaultValue);
    }

    private static UnitPref<Per<MoUnits.EncoderAngle, Distance>> encoderTicksPerCentimeterPref(
            String key, Measure<Per<MoUnits.EncoderAngle, Distance>> defaultValue) {
        return getInstance().new UnitPref<>(key, MoUnits.EncoderTicksPerCentimeter, defaultValue);
    }

    private static UnitPref<Per<MoUnits.EncoderAngle, Distance>> encoderTicksPerMeterPref(
            String key, Measure<Per<MoUnits.EncoderAngle, Distance>> defaultValue) {
        return getInstance().new UnitPref<>(key, MoUnits.EncoderTicksPerMeter, defaultValue);
    }

    private static UnitPref<Per<MoUnits.EncoderAngle, Angle>> encoderTicksPerRotationPref(
            String key, Measure<Per<MoUnits.EncoderAngle, Angle>> defaultValue) {
        return getInstance().new UnitPref<>(key, MoUnits.EncoderTicksPerRotation, defaultValue);
    }

    private static UnitPref<Time> secondsPref(String key, Measure<Time> defaultValue) {
        return getInstance().new UnitPref<>(key, Units.Seconds, defaultValue);
    }

    private static UnitPref<Current> amperesPref(String key, Measure<Current> defaultValue) {
        return getInstance().new UnitPref<>(key, Units.Amps, defaultValue);
    }
}
