package frc.robot.command;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.component.IntakeSetpointManager;
import frc.robot.component.IntakeSetpointManager.IntakeSetpoint;
import frc.robot.subsystem.IntakeSubsystem;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.util.MoPrefs;

public class HandoffCommand extends Command {
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;

    private final Timer currentSenseTimer = new Timer();

    private boolean currentTrip = false;

    public HandoffCommand(IntakeSubsystem intake, ShooterSubsystem shooter) {
        this.intake = intake;
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        currentSenseTimer.restart();
        currentTrip = false;
    }

    @Override
    public void execute() {
        shooter.setFlywheelSpeed(Units.MetersPerSecond.zero());

        var intakeHandoffPos = IntakeSetpointManager.getInstance().getSetpoint(IntakeSetpoint.HANDOFF);
        double intakeVar = MoPrefs.intakeSetpointVarianceThreshold.get().in(Units.Value);

        intake.deploySmartMotion(intakeHandoffPos);

        if (!intake.getDeployPosition().isNear(intakeHandoffPos, intakeVar)) {
            intake.rollerIntakeDirectPower(0);
            shooter.setRollerVelocity(Units.MetersPerSecond.zero());
            return;
        }

        // At this point the Arm, Intake, and Shooter are aligned!

        intake.rollerIntakeDirectPower(-MoPrefs.handoffIntakeRollerPower.get().in(Units.Value));

        if (Math.abs(shooter.getAvgFlywheelVelocity().in(Units.MetersPerSecond)) > MoPrefs.backoffZeroTolerance.get()) {
            currentTrip = true;
            intake.setIsHoldingNote(false);
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
