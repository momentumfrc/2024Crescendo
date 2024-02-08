// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.input;

import frc.robot.math.Vec2;

/** Accesses the state of robot input */
public interface MoInput {
    Vec2 getMoveRequest();

    double getTurnRequest();

    public boolean getShouldUseSlowSpeed();

    public boolean getReZeroGyro();

    public double getShootSpeed();
}
