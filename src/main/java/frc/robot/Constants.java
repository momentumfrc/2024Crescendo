// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import java.nio.file.Path;
import java.nio.file.Paths;

public class Constants {
    /**
     * A wrapper class holding a single integer value representing a CAN Address. The point of this
     * class is to indicate that the wrapped value is a CAN Address in a more robust way than just
     * adding "CAN_ADDR" to the constant's name.
     */
    public static record CANAddress(int address) {}

    /**
     * A wrapper class holding a single integer value representing a HID Port. The point of this
     * class is to indicate that the wrapped value is a HID Port in a more robust way than just
     * adding "PORT" to the constant's name.
     */
    public static record HIDPort(int port) {}

    public static final double FLOAT_EPSILON = 1e-6;

    public static final CANAddress TURN_LEFT_FRONT = new CANAddress(11);
    public static final CANAddress TURN_LEFT_REAR = new CANAddress(13);
    public static final CANAddress TURN_RIGHT_FRONT = new CANAddress(5);
    public static final CANAddress TURN_RIGHT_REAR = new CANAddress(12);
    public static final CANAddress DRIVE_LEFT_FRONT = new CANAddress(2);
    public static final CANAddress DRIVE_LEFT_REAR = new CANAddress(3);
    public static final CANAddress DRIVE_RIGHT_FRONT = new CANAddress(1);
    public static final CANAddress DRIVE_RIGHT_REAR = new CANAddress(4);

    public static final CANAddress SHOULDER_LEFT_MTR = new CANAddress(9);
    public static final CANAddress SHOULDER_RIGHT_MTR = new CANAddress(4);
    public static final CANAddress WRIST_MTR = new CANAddress(2);

    public static final CANAddress SHOOTER_ROLLER_MTR = new CANAddress(3);
    public static final CANAddress SHOOTER_FLYWHEEL_MTR_LEFT = new CANAddress(15);
    public static final CANAddress SHOOTER_FLYWHEEL_MTR_RIGHT = new CANAddress(16);

    // TODO: Placeholder id - fill with real values
    public static final CANAddress CLIMBER_LEFT = new CANAddress(0);
    public static final CANAddress CLIMBER_RIGHT = new CANAddress(0);

    // TODO: Placeholder id - fill with real values
    public static final CANAddress INTAKE_ROLLER = new CANAddress(0);
    public static final CANAddress INTAKE_DEPLOY = new CANAddress(0);

    public static final HIDPort DRIVE_F310 = new HIDPort(0);
    public static final HIDPort ARM_F310 = new HIDPort(1);
    public static final HIDPort JOYSTICK = new HIDPort(2);

    public static Path DATA_STORE_FILE;

    static {
        if (RobotBase.isReal()) {
            DATA_STORE_FILE = Paths.get("/home/lvuser/pid_constants.ini");
        } else {
            DATA_STORE_FILE = Paths.get("./pid_constants.ini");
        }
    }

    private Constants() {
        throw new UnsupportedOperationException();
    }
}
