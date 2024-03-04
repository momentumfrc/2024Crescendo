package frc.robot.util;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.HashMap;

public class SetpointDataStore {
    private static final double DEFAULT_VALUE = 0;
    private static HashMap<NetworkTable, SetpointDataStore> instances = new HashMap<>();

    public static SetpointDataStore getInstance(String tableKey) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(tableKey);
        if (!instances.containsKey(table)) {
            instances.put(table, new SetpointDataStore(table));
        }

        return instances.get(table);
    }

    private final NetworkTable table;
    private final HashMap<String, DoubleEntry> entries;

    private SetpointDataStore(NetworkTable table) {
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
