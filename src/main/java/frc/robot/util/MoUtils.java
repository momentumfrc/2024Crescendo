// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Per;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;

public class MoUtils {
    private static final double ENCODER_ZERO_ZONE = 0.2;

    public static void setupRelativeEncoder(
            RelativeEncoder relEncoder,
            Measure<Angle> absPos,
            Measure<Angle> absZero,
            Measure<Per<MoUnits.EncoderAngle, Angle>> ratio) {
        relEncoder.setPositionConversionFactor(1 / ratio.in(MoUnits.EncoderTicksPerRotation));
        relEncoder.setVelocityConversionFactor(1 / ratio.in(MoUnits.EncoderTicksPerRotation));

        double pos = absPos.in(Units.Rotations);
        pos = (pos + 1 - absZero.in(Units.Rotations)) % 1;
        if (pos > (1 - ENCODER_ZERO_ZONE)) {
            pos -= 1;
        }
        relEncoder.setPosition(pos);
    }

    public static double rotToRad(double rot) {
        return 2 * Math.PI * (rot - 0.5);
    }

    public static double radToRot(double rad) {
        return (rad / (2 * Math.PI)) + 0.5;
    }

    public static double clamp(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }

    public static <U extends Unit<U>> Measure<U> clampUnit(
            Measure<U> value, Measure<U> lowerBound, Measure<U> upperBound) {
        if (value.lt(lowerBound)) {
            return lowerBound;
        }
        if (value.gt(upperBound)) {
            return upperBound;
        }
        return value;
    }

    public static double map(double val, double inmin, double inmax, double outmin, double outmax) {
        return (((val - inmin) / (inmax - inmin)) * (outmax - outmin)) + outmin;
    }

    public static double deadzone(double val, double deadzone) {
        if (val < -deadzone) return map(val, -1, -deadzone, -1, 0);
        else if (val > deadzone) return map(val, deadzone, 1, 0, 1);
        else return 0;
    }

    public static double curve(double val, double curve) {
        if (curve == 0) return val;
        double powed = Math.pow(Math.abs(val), curve);
        if (val * powed > 0) return powed;
        else return -powed;
    }
}
