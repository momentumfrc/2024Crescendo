package frc.robot.component;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import frc.robot.subsystem.ArmSubsystem.ArmPosition;
import frc.robot.util.SetpointDataStore;
import java.util.EnumMap;

public class ArmSetpointManager {

    public static enum ArmSetpoint {
        STOW,
        HANDOFF,
        SPEAKER,
        AMP,
        SOURCE,
        SHUTTLE
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

        private final SetpointDataStore store;

        private final MutableMeasure<Angle> shoulderAngle = MutableMeasure.zero(Units.Rotations);
        private final MutableMeasure<Angle> wristAngle = MutableMeasure.zero(Units.Rotations);

        public ArmSetpointEntry(String key, SetpointDataStore store) {
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

    private SetpointDataStore store = SetpointDataStore.getInstance("Arm_Setpoints");
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
