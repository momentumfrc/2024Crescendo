// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;

public class MoPIDF extends PIDController {
    public MoPIDF() {
        super(0, 0, 0);
    }

    private double kF = 0;

    private double setpoint = 0;
    private double lastMeasurement = 0;
    private double lastOutput = 0;

    public double calculate(double measurement, double setpoint) {
        this.setpoint = setpoint;
        this.lastMeasurement = measurement;
        lastOutput = super.calculate(measurement, setpoint) + (kF * setpoint);
        return lastOutput;
    }

    public void setFF(double kFF) {
        kF = kFF;
    }

    public void setIZone(double kIZone) {
        super.setIntegratorRange(-kIZone, kIZone);
    }

    public double getSetpoint() {
        return setpoint;
    }

    public double getLastMeasurement() {
        return lastMeasurement;
    }

    public double getLastOutput() {
        return lastOutput;
    }
}
