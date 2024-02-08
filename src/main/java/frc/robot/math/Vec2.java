// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.math;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import java.util.function.DoubleUnaryOperator;
import org.ejml.simple.SimpleMatrix;

/** Vector in 2 dimensions */
public class Vec2 extends Vector<N2> {
    public Vec2(Matrix<N2, N1> mat) {
        super(mat);
    }

    public Vec2(SimpleMatrix mat) {
        super(mat);
    }

    public Vec2(double x, double y) {
        this(new Matrix<>(N2.instance, N1.instance));
        set(0, 0, x);
        set(1, 0, y);
    }

    public Vec2(Vector<N2> vec) {
        this(vec.getStorage());
    }

    public double x() {
        return get(0, 0);
    }

    public double y() {
        return get(1, 0);
    }

    public void setX(double x) {
        set(0, 0, x);
    }

    public void setY(double y) {
        set(1, 0, y);
    }

    /** Alias for {@code norm()} */
    public double len() {
        return norm();
    }

    public Vec2 normalized() {
        double len = len();
        if (len > 0) {
            return new Vec2(div(len));
        }

        return new Vec2(0, 0);
    }

    /*
     * Change the magnitude of the vector without changing the direction
     */
    public Vec2 scalarOp(DoubleUnaryOperator op) {
        double len = len();

        if (len > 0) {
            return new Vec2(div(len).times(op.applyAsDouble(len)));
        }

        return new Vec2(0, 0);
    }
}
