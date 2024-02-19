// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.component;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.momentum4999.motune.PIDTuner;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkAnalogSensor.Mode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Per;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.util.MoPrefs.UnitPref;
import frc.robot.util.MoSparkMaxPID;
import frc.robot.util.MoTalonFxPID;
import frc.robot.util.MoUnits;
import frc.robot.util.MoUtils;
import frc.robot.util.TunerUtils;
import java.util.Map;

public class SwerveModule {
    private static final double ABSOLUTE_ENCODER_SCALE = 1 / 3.3;
    private static final double MOTOR_UNPOWERED_SPEED = 0.05;

    private final String key;
    public final CANSparkMax turnMotor;
    public final TalonFX driveMotor;

    // Note: the absolute encoder returns rotations, in the range [0, 1)
    public final SparkAnalogSensor absoluteEncoder;

    public final MoSparkMaxPID turnPID;
    private final MoTalonFxPID drivePID;

    private PIDTuner turnTuner;
    private PIDTuner driveTuner;

    // Note: the relative encoder is scaled to return radians
    public final RelativeEncoder relativeEncoder;

    private final UnitPref<Angle> encoderZero;
    private final UnitPref<Per<Dimensionless, Angle>> encoderRotScale;
    private final UnitPref<Per<Dimensionless, Distance>> encoderDistScale;

    public SwerveModule(
            String key,
            CANSparkMax turnMotor,
            TalonFX driveMotor,
            UnitPref<Angle> encoderZero,
            UnitPref<Per<Dimensionless, Angle>> encoderRotScale,
            UnitPref<Per<Dimensionless, Distance>> encoderDistScale) {
        this.key = key;
        this.turnMotor = turnMotor;
        this.driveMotor = driveMotor;
        this.encoderZero = encoderZero;
        this.encoderRotScale = encoderRotScale;
        this.encoderDistScale = encoderDistScale;

        this.driveMotor.setNeutralMode(NeutralModeValue.Brake);
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

        encoderZero.subscribe(zero -> this.setupRelativeEncoder(getAbsoluteRotation(), zero, encoderRotScale.get()));
        encoderRotScale.subscribe(scale -> this.setupRelativeEncoder(getAbsoluteRotation(), encoderZero.get(), scale));
        setupRelativeEncoder();

        var layout = Shuffleboard.getTab("match")
                .getLayout(key, BuiltInLayouts.kList)
                .withSize(2, 1)
                .withProperties(Map.of("Label position", "LEFT"));
        layout.addDouble("Relative", () -> (MoUtils.radToRot(relativeEncoder.getPosition())));
        layout.addDouble("Absolute", absoluteEncoder::getPosition);
    }

    public Measure<Angle> getAbsoluteRotation() {
        return Units.Rotations.of(absoluteEncoder.getPosition());
    }

    public Measure<Angle> getRelativeRotation() {
        return Units.Radians.of(relativeEncoder.getPosition());
    }

    public Measure<Distance> getDistance() {
        return Units.Meters.of(driveMotor.getRotorPosition().getValueAsDouble()
                / encoderDistScale.get().in(MoUnits.EncoderTicksPerMeter));
    }

    public Measure<Velocity<Distance>> getVelocity() {
        return Units.MetersPerSecond.of(driveMotor.getRotorVelocity().getValueAsDouble()
                / encoderDistScale.get().in(MoUnits.EncoderTicksPerMeter));
    }

    private boolean areMotorsPowered() {
        return Math.abs(driveMotor.get()) > MOTOR_UNPOWERED_SPEED && Math.abs(turnMotor.get()) > MOTOR_UNPOWERED_SPEED;
    }

    public void setupRelativeEncoder() {
        setupRelativeEncoder(getAbsoluteRotation(), encoderZero.get(), encoderRotScale.get());
    }

    public void setRelativePosition() {
        if (!areMotorsPowered()) {
            setRelativePosition(getAbsoluteRotation(), encoderZero.get());
        }
    }

    private void setupRelativeEncoder(
            Measure<Angle> absPos, Measure<Angle> absZero, Measure<Per<Dimensionless, Angle>> scale) {
        relativeEncoder.setPositionConversionFactor(1 / scale.in(MoUnits.EncoderTicksPerRadian));
        relativeEncoder.setVelocityConversionFactor(1 / scale.in(MoUnits.EncoderTicksPerRadian));

        setRelativePosition(absPos, absZero);
    }

    private void setRelativePosition(Measure<Angle> absPos, Measure<Angle> absZero) {
        double rots = absPos.in(Units.Rotations);
        rots = (rots + 1 - absZero.in(Units.Rotations)) % 1;
        relativeEncoder.setPosition(MoUtils.rotToRad(rots));
    }

    public void drive(SwerveModuleState state) {
        var optimized = SwerveModuleState.optimize(
                state, Rotation2d.fromRadians(getRelativeRotation().in(Units.Radians)));
        turnPID.setReference(MathUtil.angleModulus(optimized.angle.getRadians()));
        drivePID.setReference(
                optimized.speedMetersPerSecond * encoderDistScale.get().in(MoUnits.EncoderTicksPerMeter));
    }

    public void directDrive(double turnSpeed, double driveSpeed) {
        turnMotor.set(turnSpeed);
        driveMotor.setControl(new DutyCycleOut(driveSpeed));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                getDistance(), Rotation2d.fromRadians(getRelativeRotation().in(Units.Radians)));
    }

    @Override
    public String toString() {
        return String.format("SwerveModule(key=\"%s\")", this.key);
    }
}
