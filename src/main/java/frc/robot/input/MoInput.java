// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.input;

import frc.robot.component.ArmSetpointManager.ArmSetpoint;
import frc.robot.math.Vec2;
import frc.robot.subsystem.ArmSubsystem.ArmMovementRequest;
import java.util.Optional;

/** Accesses the state of robot input */
public interface MoInput {
    Vec2 getMoveRequest();

    double getTurnRequest();

    public boolean getShouldUseSlowSpeed();

    public boolean getReZeroGyro();

    public ArmMovementRequest getArmMovementRequest();

    public Optional<ArmSetpoint> getArmSetpoint();

    public boolean getSaveArmSetpoint();
}
