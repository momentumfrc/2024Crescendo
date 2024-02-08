package frc.robot.subsystem;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private CANSparkFlex upperMtr = new CANSparkFlex(16, MotorType.kBrushless);
    private CANSparkFlex lwrMtr = new CANSparkFlex(15, MotorType.kBrushless);

    public ShooterSubsystem() {
        upperMtr.setInverted(true);
        lwrMtr.setInverted(false);
        upperMtr.follow(lwrMtr);
    }

    public void setSpeed(double spd) {
        lwrMtr.set(spd);
    }
}
