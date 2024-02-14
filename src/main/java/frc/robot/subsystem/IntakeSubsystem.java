package frc.robot.subsystem;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private static final int HARD_CURRENT_LIMIT = 20;

    private final CANSparkMax intakeMtr;

    private final DoublePublisher currentPublisher;

    public IntakeSubsystem() {
        intakeMtr = new CANSparkMax(Constants.INTAKE_MTR.address, MotorType.kBrushless);
        intakeMtr.setSmartCurrentLimit(HARD_CURRENT_LIMIT);

        currentPublisher = NetworkTableInstance.getDefault()
                .getTable("robot_state")
                .getDoubleTopic("intake_current")
                .publish();
    }

    public void set(double speed) {
        intakeMtr.set(speed);
    }

    public Measure<Current> getCurrent() {
        return Units.Amps.of(intakeMtr.getOutputCurrent());
    }

    @Override
    public void periodic() {
        currentPublisher.accept(intakeMtr.getOutputCurrent());
    }
}
