// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.component;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.momentum4999.motune.PIDTuner;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkAnalogSensor.Mode;
import com.revrobotics.SparkMaxAnalogSensor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.util.MoPrefs;
import frc.robot.util.MoSparkMaxPID;
import frc.robot.util.MoTalonFxPID;
import frc.robot.util.MoUtils;
import frc.robot.util.TunerUtils;
import frc.robot.util.MoPrefs.Pref;
import frc.robot.util.MoSparkMaxPID.Type;

public class SwerveModule {
    private static final double ABSOLUTE_ENCODER_SCALE = 1/3.3;
    private static final double MOTOR_UNPOWERED_SPEED = 0.05;

    private final String key;
    public final CANSparkMax turnMotor;
    public final WPI_TalonFX driveMotor;

    // Note: the absolute encoder returns rotations, in the range [0, 1)
    public final SparkMaxAnalogSensor absoluteEncoder;

    private final MoSparkMaxPID turnPID;
    private final MoTalonFxPID drivePID;

    private PIDTuner turnTuner;
    private PIDTuner driveTuner;

    // Note: the relative encoder is scaled to return radians
    public final RelativeEncoder relativeEncoder;

    private final Pref<Double> encoderZero;
    private final Pref<Double> encoderScale;
    private final Pref<Double> driveMtrScale;

    public SwerveModule(String key, CANSparkMax turnMotor, WPI_TalonFX driveMotor, Pref<Double> encoderZero, Pref<Double> encoderScale, Pref<Double> driveMtrScale) {
        this.key = key;
        this.turnMotor = turnMotor;
        this.driveMotor = driveMotor;
        this.encoderZero = encoderZero;
        this.encoderScale = encoderScale;
        this.driveMtrScale = driveMtrScale;

        this.driveMotor.setNeutralMode(NeutralMode.Brake);
        this.turnMotor.setIdleMode(IdleMode.kBrake);

        this.absoluteEncoder = turnMotor.getAnalog(Mode.kAbsolute);
        this.absoluteEncoder.setPositionConversionFactor(ABSOLUTE_ENCODER_SCALE);

        this.turnPID = new MoSparkMaxPID(MoSparkMaxPID.Type.POSITION, turnMotor, 0);
        this.drivePID = new MoTalonFxPID(MoTalonFxPID.Type.VELOCITY, driveMotor);

        var turnSparkMaxPID = turnPID.getPID();
        turnSparkMaxPID.setPositionPIDWrappingMinInput(-Math.PI);
        turnSparkMaxPID.setPositionPIDWrappingMaxInput(Math.PI);
        turnSparkMaxPID.setPositionPIDWrappingEnabled(true);

        turnTuner = TunerUtils.forMoSparkMax(turnPID, key + "_turn");
        driveTuner = TunerUtils.forMoTalonFx(drivePID, key + "_drive");

        relativeEncoder = turnMotor.getEncoder();

        encoderZero.subscribe(zero -> this.setupRelativeEncoder(absoluteEncoder.getPosition(), zero, encoderScale.get()), false);
        encoderScale.subscribe(scale -> this.setupRelativeEncoder(absoluteEncoder.getPosition(), encoderZero.get(), scale), false);
        setupRelativeEncoder();

        var layout = Shuffleboard.getTab("match").getLayout(key, BuiltInLayouts.kList)
            .withSize(2, 1)
            .withProperties(Map.of("Label position", "LEFT"));
        layout.addDouble("Relative", () -> (MoUtils.radToRot(relativeEncoder.getPosition())));
        layout.addDouble("Absolute", absoluteEncoder::getPosition);
    }

    private boolean areMotorsPowered() {
        return driveMotor.get() > MOTOR_UNPOWERED_SPEED
            && turnMotor.get() > MOTOR_UNPOWERED_SPEED;
    }

    public void setupRelativeEncoder() {
        setupRelativeEncoder(absoluteEncoder.getPosition(), encoderZero.get(), encoderScale.get());
    }

    public void setRelativePosition() {
        if(!areMotorsPowered())
            setRelativePosition(absoluteEncoder.getPosition(), encoderZero.get());
    }

    private void setupRelativeEncoder(double absPos, double absZero, double scale) {
        relativeEncoder.setPositionConversionFactor(scale);
        relativeEncoder.setVelocityConversionFactor(scale);

        setRelativePosition(absPos, absZero);
    }

    private void setRelativePosition(double absPos, double absZero) {
        double rots = absPos;
        rots = (rots + 1 - absZero) % 1;
        relativeEncoder.setPosition(MoUtils.rotToRad(rots));
    }

    public void drive(SwerveModuleState state) {
        var optimized = SwerveModuleState.optimize(state, Rotation2d.fromRadians(relativeEncoder.getPosition()));
        turnPID.setReference(MathUtil.angleModulus(optimized.angle.getRadians()));
        drivePID.setReference(optimized.speedMetersPerSecond * driveMtrScale.get());
    }

    public void directDrive(double turnSpeed, double driveSpeed) {
        turnMotor.set(turnSpeed);
        driveMotor.set(ControlMode.PercentOutput, driveSpeed);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getSelectedSensorPosition() / driveMtrScale.get(), Rotation2d.fromRadians(relativeEncoder.getPosition()));
    }

    @Override
    public String toString() {
        return String.format("SwerveModule(key=\"%s\")", this.key);
    }
}
