package frc.robot.command;

import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.component.ArmSetpointManager;
import frc.robot.component.ArmSetpointManager.ArmSetpoint;
import frc.robot.subsystem.ArmSubsystem;
import frc.robot.subsystem.ArmSubsystem.ArmPosition;
import frc.robot.subsystem.PositioningSubsystem;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.util.MoPrefs;
import frc.robot.util.MoShuffleboard;
import frc.robot.util.MoUnits;
import java.util.Map;

public class TuneShooterCommand extends Command {
    private final ShooterSubsystem shooter;
    private final ArmSubsystem arm;
    private final PositioningSubsystem pos;

    private GenericSubscriber wristAngleSubscriber;
    private MutableMeasure<Angle> wristAngleMeasure = MutableMeasure.zero(Units.Rotations);

    public TuneShooterCommand(ShooterSubsystem shooter, ArmSubsystem arm, PositioningSubsystem pos) {
        this.shooter = shooter;
        this.arm = arm;
        this.pos = pos;

        var aimGroup = MoShuffleboard.getInstance()
                .shooterTab
                .getLayout("Aim Speaker", BuiltInLayouts.kList)
                .withSize(2, 1)
                .withProperties(Map.of("Label position", "RIGHT"));
        aimGroup.addDouble("Speaker Dist.", () -> pos.getSpeakerPose()
                .getTranslation()
                .getDistance(pos.getRobotPose().getTranslation()));
        wristAngleSubscriber = aimGroup.add("Wrist Angle (R)", 0).getEntry();

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        // double thresh = MoPrefs.shooterSetpointVarianceThreshold.get().in(Units.Value);

        ArmPosition position = ArmSetpointManager.getInstance().getSetpoint(ArmSetpoint.SPEAKER);
        position = new ArmPosition(
                position.shoulderAngle(),
                wristAngleMeasure.mut_replace(
                        wristAngleSubscriber.getDouble(position.shoulderAngle().in(Units.Rotations)), Units.Rotations));

        arm.adjustSmartPosition(position);

        if (!arm.atPosition(position)) {
            shooter.setFlywheelSpeed(MoUnits.CentimetersPerSec.zero());
            shooter.setRollerVelocity(MoUnits.CentimetersPerSec.zero());
            return;
        }

        var flywheelSetpoint = MoPrefs.flywheelSpeakerSetpoint.get();
        shooter.setFlywheelSpeed(flywheelSetpoint);

        if (shooter.getAvgFlywheelVelocity().gte(flywheelSetpoint.minus(MoPrefs.shooterSetpointThreshold.get()))) {
            shooter.setRollerVelocity(flywheelSetpoint);
        } else {
            shooter.setRollerVelocity(MoUnits.CentimetersPerSec.zero());
        }
    }
}
