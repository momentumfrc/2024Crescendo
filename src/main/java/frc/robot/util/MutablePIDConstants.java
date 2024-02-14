// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.pathplanner.lib.util.PIDConstants;

public class MutablePIDConstants {
    public double kP, kI, kD, iZone;

    public MutablePIDConstants() {
        this.iZone = 1.0;
    }

    public PIDConstants toImmutable() {
        return new PIDConstants(kP, kI, kD, iZone);
    }
}
