// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.momentum4999.motune.PIDTuner;
import com.momentum4999.motune.PIDTunerBuilder;
import com.playingwithfusion.CANVenom;

import frc.robot.Constants;

public class TunerUtils {
    public static PIDTuner forMoPIDF(MoPIDF controller, String controllerName) {
        return PIDTuner.builder(controllerName)
            .withDataStoreFile(Constants.DATA_STORE_FILE)
            .withP(controller::setP)
            .withI(controller::setI)
            .withD(controller::setD)
            .withFF(controller::setFF)
            .withIZone(controller::setIZone)
            .withSetpoint(controller::getSetpoint)
            .withMeasurement(controller::getLastMeasurement)
            .safeBuild();
    }

    public static PIDTuner forMoPID(MoPIDF controller, String controllerName) {
        return PIDTuner.builder(controllerName)
            .withDataStoreFile(Constants.DATA_STORE_FILE)
            .withP(controller::setP)
            .withI(controller::setI)
            .withD(controller::setD)
            .withIZone(controller::setIZone)
            .withSetpoint(controller::getSetpoint)
            .withMeasurement(controller::getLastMeasurement)
            .safeBuild();
    }

    public static PIDTuner forVenom(CANVenom venom, String controllerName) {
        return PIDTuner.builder(controllerName)
            .withDataStoreFile(Constants.DATA_STORE_FILE)
            .withP(venom::setKP)
            .withI(venom::setKI)
            .withD(venom::setKD)
            .withFF(venom::setKF)
            .withSetpoint(venom::getPIDTarget)
            .withMeasurement(venom::getSpeed)
            .safeBuild();
    }

    public static PIDTuner forMoSparkMax(MoSparkMaxPID sparkMax, String controllerName) {
        PIDTunerBuilder builder = PIDTuner.builder(controllerName)
            .withDataStoreFile(Constants.DATA_STORE_FILE)
            .withP(sparkMax::setP)
            .withI(sparkMax::setI)
            .withD(sparkMax::setD)
            .withFF(sparkMax::setFF)
            .withIZone(sparkMax::setIZone)
            .withSetpoint(sparkMax::getSetpoint)
            .withMeasurement(sparkMax::getLastMeasurement);

        if(sparkMax.getType() == MoSparkMaxPID.Type.SMARTMOTION) {
            builder = builder
                .withProperty("maxVel", (v) -> sparkMax.getPID().setSmartMotionMaxVelocity(v, sparkMax.getPidSlot()))
                .withProperty("maxAccel", (a) -> sparkMax.getPID().setSmartMotionMaxAccel(a, sparkMax.getPidSlot()))
                .withProperty("allowedError", (e) -> sparkMax.getPID().setSmartMotionAllowedClosedLoopError(e, sparkMax.getPidSlot()));
        }

        return builder.safeBuild();
    }

    public static PIDTuner forMoTalonFx(MoTalonFxPID talon, String controllerName) {
        return PIDTuner.builder(controllerName)
            .withDataStoreFile(Constants.DATA_STORE_FILE)
            .withP(talon::setP)
            .withI(talon::setI)
            .withD(talon::setD)
            .withFF(talon::setFF)
            .withIZone(talon::setIZone)
            .withSetpoint(talon::getSetpoint)
            .withMeasurement(talon::getLastMeasurement)
            .safeBuild();
    }
}
