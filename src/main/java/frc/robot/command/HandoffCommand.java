package frc.robot.command;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.component.ArmSetpointManager;
import frc.robot.component.ArmSetpointManager.ArmSetpoint;
import frc.robot.component.IntakeSetpointManager;
import frc.robot.component.IntakeSetpointManager.IntakeSetpoint;
import frc.robot.subsystem.ArmSubsystem;
import frc.robot.subsystem.ArmSubsystem.ArmPosition;
import frc.robot.subsystem.IntakeSubsystem;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.util.MoPrefs;

public class HandoffCommand extends Command {
    private final ArmSubsystem arm;
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;

    private final Timer currentSenseTimer = new Timer();

    private boolean currentTrip = false;

    public HandoffCommand(ArmSubsystem arm, IntakeSubsystem intake, ShooterSubsystem shooter) {
        this.arm = arm;
        this.intake = intake;
        this.shooter = shooter;

        addRequirements(arm, intake, shooter);
    }

    @Override
    public void initialize() {
        currentSenseTimer.restart();
        currentTrip = false;
    }

    @Override
    public void execute() {
        shooter.setFlywheelSpeed(Units.MetersPerSecond.zero());

        ArmPosition armPos = ArmSetpointManager.getInstance().getSetpoint(ArmSetpoint.HANDOFF);
        Measure<Angle> intakePos = IntakeSetpointManager.getInstance().getSetpoint(IntakeSetpoint.HANDOFF);

        arm.adjustSmartPosition(armPos);
        intake.deploySmartMotion(intakePos);

        double intakeTolerance = MoPrefs.intakeSetpointVarianceThreshold.get().in(Units.Value);

        if (!arm.atPosition(armPos) || !intake.getDeployPosition().isNear(intakePos, intakeTolerance)) {
            shooter.setRollerVelocity(Units.MetersPerSecond.zero());
            intake.intakeDirectPower(0);

            return;
        }

        // At this point, the intake and arm are aligned.
        intake.intakeDirectPower(-MoPrefs.handoffIntakeRollerPower.get().in(Units.Value));

        if (shooter.getRollerCurrent().gte(MoPrefs.handoffCurrentCutoff.get())) {
            if (currentSenseTimer.hasElapsed(MoPrefs.handoffTimeCutoff.get().in(Units.Seconds))) {
                currentTrip = true;
            }
        } else {
            currentSenseTimer.restart();
        }

        if (currentTrip) {
            shooter.setRollerVelocity(Units.MetersPerSecond.of(0));
        } else {
            shooter.directDriveRoller(MoPrefs.handoffShooterRollerPower.get().in(Units.Value));
        }
    }

    @Override
    public boolean isFinished() {
        return currentTrip;
    }
}
