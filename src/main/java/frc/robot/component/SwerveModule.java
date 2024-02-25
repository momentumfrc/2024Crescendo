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
import com.revrobotics.SparkAnalogSensor.Mode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Per;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.encoder.MoEncoder;
import frc.robot.util.MoPrefs.UnitPref;
import frc.robot.util.MoSparkMaxPID;
import frc.robot.util.MoTalonFxPID;
import frc.robot.util.MoUnits;
import frc.robot.util.TunerUtils;
import java.util.Map;

public class SwerveModule {
    // The Thrifty absolute magnetic encoder outputs a voltage between 0-5v throughout 1 rotation, but it's scaled
    // down to 1-3.3v by the SparkMax data port breakout board. Thus, when the mechanism travels 1 rotation, the
    // absolute encoder will travel through 3.3 "Encoder Ticks" (which happen to correspond to volts).
    private static final Measure<Per<MoUnits.EncoderAngle, Angle>> ABSOLUTE_ENCODER_SCALE =
            MoUnits.EncoderTicksPerRotation.of(3.3);

    private static final double MOTOR_UNPOWERED_SPEED = 0.05;

    private final String key;
    public final CANSparkMax turnMotor;
    public final TalonFX driveMotor;

    // Note: the absolute encoder returns rotations, in the range [0, 1)
    public final MoEncoder<Angle> absoluteEncoder;
    public final MoEncoder<Angle> relativeEncoder;

    public final MoEncoder<Distance> distEncoder;

    public final MoSparkMaxPID<Angle> turnPID;
    private final MoTalonFxPID<Distance> drivePID;

    private PIDTuner turnTuner;
    private PIDTuner driveTuner;

    private final UnitPref<Angle> encoderZero;
    private final UnitPref<Per<MoUnits.EncoderAngle, Angle>> encoderRotScale;

    // Keep references to frequently used measure instances, so that we're not constantly creating/destroying instances.
    // This reduces the amount of garbage collection that is needed. See:
    // https://docs.wpilib.org/en/stable/docs/software/basic-programming/java-units.html#mutability-and-object-creation
    private final MutableMeasure<Angle> mut_angleSetpoint = MutableMeasure.zero(Units.Rotations);
    private final MutableMeasure<Velocity<Distance>> mut_velocitySetpoint = MutableMeasure.zero(Units.MetersPerSecond);
    private final MutableMeasure<Distance> distance = MutableMeasure.zero(Units.Meters);
    private final MutableMeasure<Velocity<Distance>> velocity = MutableMeasure.zero(Units.MetersPerSecond);

    public SwerveModule(
            String key,
            CANSparkMax turnMotor,
            TalonFX driveMotor,
            UnitPref<Angle> encoderZero,
            UnitPref<Per<MoUnits.EncoderAngle, Angle>> encoderRotScale,
            UnitPref<Per<MoUnits.EncoderAngle, Distance>> encoderDistScale) {
        this.key = key;
        this.turnMotor = turnMotor;
        this.driveMotor = driveMotor;
        this.encoderZero = encoderZero;
        this.encoderRotScale = encoderRotScale;

        this.driveMotor.setNeutralMode(NeutralModeValue.Brake);
        this.turnMotor.setIdleMode(IdleMode.kBrake);

        this.absoluteEncoder = MoEncoder.forSparkAnalog(turnMotor.getAnalog(Mode.kAbsolute), Units.Rotations);
        this.absoluteEncoder.setConversionFactor(ABSOLUTE_ENCODER_SCALE);

        relativeEncoder = MoEncoder.forSparkRelative(turnMotor.getEncoder(), Units.Radians);

        distEncoder = MoEncoder.forTalonFx(driveMotor, Units.Meters);
        encoderDistScale.subscribe(scale -> distEncoder.setConversionFactor(scale));

        this.turnPID = new MoSparkMaxPID<Angle>(
                MoSparkMaxPID.Type.POSITION, turnMotor, 0, relativeEncoder.getInternalEncoderUnits());
        this.drivePID = new MoTalonFxPID<Distance>(
                MoTalonFxPID.Type.VELOCITY, driveMotor, distEncoder.getInternalEncoderUnits());

        var turnSparkMaxPID = turnPID.getPID();
        turnSparkMaxPID.setPositionPIDWrappingMinInput(-Math.PI);
        turnSparkMaxPID.setPositionPIDWrappingMaxInput(Math.PI);
        turnSparkMaxPID.setPositionPIDWrappingEnabled(true);

        turnTuner = TunerUtils.forMoSparkMax(turnPID, key + "_turn");
        driveTuner = TunerUtils.forMoTalonFx(drivePID, key + "_drive");

        encoderZero.subscribe(
                zero -> this.setupRelativeEncoder(absoluteEncoder.getPosition(), zero, encoderRotScale.get()));
        encoderRotScale.subscribe(
                scale -> this.setupRelativeEncoder(absoluteEncoder.getPosition(), encoderZero.get(), scale));
        setupRelativeEncoder();

        var layout = Shuffleboard.getTab("match")
                .getLayout(key, BuiltInLayouts.kList)
                .withSize(2, 1)
                .withProperties(Map.of("Label position", "LEFT"));
        layout.addDouble("Relative", () -> relativeEncoder.getPosition().in(Units.Rotations));
        layout.addDouble("Absolute", () -> absoluteEncoder.getPosition().in(Units.Rotations));
    }

    private boolean areMotorsPowered() {
        return Math.abs(driveMotor.get()) > MOTOR_UNPOWERED_SPEED && Math.abs(turnMotor.get()) > MOTOR_UNPOWERED_SPEED;
    }

    public void setupRelativeEncoder() {
        setupRelativeEncoder(absoluteEncoder.getPosition(), encoderZero.get(), encoderRotScale.get());
    }

    public void setRelativePosition() {
        if (!areMotorsPowered()) {
            setRelativePosition(absoluteEncoder.getPosition(), encoderZero.get());
        }
    }

    private void setupRelativeEncoder(
            Measure<Angle> absPos, Measure<Angle> absZero, Measure<Per<MoUnits.EncoderAngle, Angle>> scale) {
        relativeEncoder.setConversionFactor(scale);
        setRelativePosition(absPos, absZero);
    }

    private void setRelativePosition(Measure<Angle> absPos, Measure<Angle> absZero) {
        double rots = absPos.in(Units.Rotations);
        rots = (rots + 1 - absZero.in(Units.Rotations)) % 1;
        relativeEncoder.setPosition(Units.Rotations.of(rots));
    }

    public void drive(SwerveModuleState state) {
        var optimized = SwerveModuleState.optimize(state, new Rotation2d(relativeEncoder.getPosition()));
        turnPID.setPositionReference(
                mut_angleSetpoint.mut_replace(MathUtil.angleModulus(optimized.angle.getRadians()), Units.Radians));
        drivePID.setVelocityReference(
                mut_velocitySetpoint.mut_replace(optimized.speedMetersPerSecond, Units.MetersPerSecond));
    }

    public void directDrive(double turnSpeed, double driveSpeed) {
        turnMotor.set(turnSpeed);
        driveMotor.setControl(new DutyCycleOut(driveSpeed));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(distEncoder.getPosition(), new Rotation2d(relativeEncoder.getPosition()));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(distEncoder.getVelocity(), new Rotation2d(relativeEncoder.getPosition()));
    }

    @Override
    public String toString() {
        return String.format("SwerveModule(key=\"%s\")", this.key);
    }
}
