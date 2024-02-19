package frc.robot.component;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import frc.robot.subsystem.ArmSubsystem.ArmPosition;
import java.util.EnumMap;
import java.util.HashMap;

public class ArmSetpointManager {

    private static class DataStore {
        private static final double DEFAULT_VALUE = 0;
        private static HashMap<NetworkTable, DataStore> instances = new HashMap<>();

        public static DataStore getInstance(String tableKey) {
            NetworkTable table = NetworkTableInstance.getDefault().getTable(tableKey);
            if (!instances.containsKey(table)) {
                instances.put(table, new DataStore(table));
            }

            return instances.get(table);
        }

        private final NetworkTable table;
        private final HashMap<String, DoubleEntry> entries;

        public DataStore(NetworkTable table) {
            this.table = table;
            entries = new HashMap<>();
        }

        private DoubleEntry getEntry(String key, double defaultValue) {
            if (!entries.containsKey(key)) {
                DoubleTopic topic = table.getDoubleTopic(key);
                DoubleEntry entry = topic.getEntry(defaultValue);
                entry.setDefault(defaultValue);
                topic.setPersistent(true);
                entries.put(key, entry);
                return entry;
            } else {
                return entries.get(key);
            }
        }

        public void putValue(String key, double value) {
            getEntry(key, value).set(value);
        }

        public double getValue(String key) {
            return getEntry(key, DEFAULT_VALUE).get();
        }
    }

    public static enum ArmSetpoint {
        STOW,
        HANDOFF,
        SPEAKER,
        AMP,
        SOURCE
    };

    private static ArmSetpointManager instance;

    public static ArmSetpointManager getInstance() {
        if (instance == null) {
            instance = new ArmSetpointManager();
        }
        return instance;
    }

    private static class ArmSetpointEntry {
        private final String shoulderKey;
        private final String wristKey;

        private final DataStore store;

        private final MutableMeasure<Angle> shoulderAngle = MutableMeasure.zero(Units.Rotations);
        private final MutableMeasure<Angle> wristAngle = MutableMeasure.zero(Units.Rotations);

        public ArmSetpointEntry(String key, DataStore store) {
            this.shoulderKey = String.format("%s_shoulder", key);
            this.wristKey = String.format("%s_wrist", key);
            this.store = store;
        }

        public ArmPosition getValue() {
            return new ArmPosition(
                    shoulderAngle.mut_replace(store.getValue(shoulderKey), Units.Rotations),
                    wristAngle.mut_replace(store.getValue(wristKey), Units.Rotations));
        }

        public void setValue(ArmPosition position) {
            store.putValue(shoulderKey, position.shoulderAngle().in(Units.Rotations));
            store.putValue(wristKey, position.wristAngle().in(Units.Rotations));
        }
    }

    private DataStore store = DataStore.getInstance("Arm_Setpoints");
    private EnumMap<ArmSetpoint, ArmSetpointEntry> entries = new EnumMap<>(ArmSetpoint.class);

    private ArmSetpointManager() {
        for (ArmSetpoint setpoint : ArmSetpoint.values()) {
            entries.put(setpoint, new ArmSetpointEntry(setpoint.name(), store));
        }
    }

    public ArmPosition getSetpoint(ArmSetpoint setpoint) {
        return entries.get(setpoint).getValue();
    }

    public void setSetpoint(ArmSetpoint setpoint, ArmPosition position) {
        entries.get(setpoint).setValue(position);
    }
}
