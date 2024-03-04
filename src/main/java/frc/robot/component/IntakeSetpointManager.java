package frc.robot.component;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import frc.robot.util.SetpointDataStore;
import java.util.EnumMap;

public class IntakeSetpointManager {

    public static enum IntakeSetpoint {
        STOW,
        HANDOFF,
        INTAKE
    };

    private static IntakeSetpointManager instance;

    public static IntakeSetpointManager getInstance() {
        if (instance == null) {
            instance = new IntakeSetpointManager();
        }
        return instance;
    }

    private static class IntakeSetpointEntry {
        private final String key;

        private final SetpointDataStore store;

        private final MutableMeasure<Angle> angle = MutableMeasure.zero(Units.Rotations);

        public IntakeSetpointEntry(String key, SetpointDataStore store) {
            this.key = key;
            this.store = store;
        }

        public Measure<Angle> getValue() {
            return angle.mut_replace(store.getValue(key), Units.Rotations);
        }

        public void setValue(Measure<Angle> position) {
            store.putValue(key, position.in(Units.Rotations));
        }
    }

    private SetpointDataStore store = SetpointDataStore.getInstance("Intake_Setpoints");
    private EnumMap<IntakeSetpoint, IntakeSetpointEntry> entries = new EnumMap<>(IntakeSetpoint.class);

    private IntakeSetpointManager() {
        for (IntakeSetpoint setpoint : IntakeSetpoint.values()) {
            entries.put(setpoint, new IntakeSetpointEntry(setpoint.name(), store));
        }
    }

    public Measure<Angle> getSetpoint(IntakeSetpoint setpoint) {
        return entries.get(setpoint).getValue();
    }

    public void setSetpoint(IntakeSetpoint setpoint, Measure<Angle> position) {
        entries.get(setpoint).setValue(position);
    }
}
