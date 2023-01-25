package frc.robot.util.coprocessor.detections;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.*;

public class Pose3d {
    public Position position = new Position();
    public Orientation orientation = new Orientation();

    public Pose3d() {  }
    public Pose3d(Position position, Orientation orientation) {
        this.position = position;
        this.orientation = orientation;
    }

    public Pose3d(
        double position_x,
        double position_y,
        double position_z,
        double orientation_w,
        double orientation_x,
        double orientation_y,
        double orientation_z
    ) {
        position.x = position_x;
        position.y = position_y;
        position.z = position_z;
        orientation.w = orientation_w;
        orientation.x = orientation_x;
        orientation.y = orientation_y;
        orientation.z = orientation_z;
    }

    public Pose3d(Matrix<N4, N4> matrix) {
        position.x = matrix.get(0, 3);
        position.y = matrix.get(1, 3);
        position.z = matrix.get(2, 3);

        double m00 = matrix.get(0, 0);
        double m01 = matrix.get(0, 1);
        double m02 = matrix.get(0, 2);
        double m10 = matrix.get(1, 0);
        double m11 = matrix.get(1, 1);
        double m12 = matrix.get(1, 2);
        double m20 = matrix.get(2, 0);
        double m21 = matrix.get(2, 1);
        double m22 = matrix.get(2, 2);
        orientation.w = Math.sqrt(1.0 + m00 + m11 + m22) / 2.0;
        double w4 = 4.0 * orientation.w;
        orientation.x = (m21 - m12) / (w4);
        orientation.y = (m02 - m20) / (w4);
        orientation.z = (m10 - m01) / (w4);
    }

    public Pose3d invert() {
        return new Pose3d(
            position.invert(),
            orientation.invert()
        );
    }

    public Pose3d relativeTo(Pose3d other) {
        Pose3d otherInvert = other.invert();
        Position relativePoint = position.translateBy(otherInvert.position);
        Position rotatedPoint = relativePoint.rotateBy(otherInvert.orientation);
        Orientation rotatedOrientation = orientation.rotateBy(otherInvert.orientation);
        return new Pose3d(rotatedPoint, rotatedOrientation);
    }

    public Pose3d transformBy(Pose3d other) {
        Position rotatedPoint = position.rotateBy(other.orientation);
        Position relativePoint = rotatedPoint.translateBy(other.position);
        Orientation rotatedOrientation = orientation.rotateBy(other.orientation);
        return new Pose3d(relativePoint, rotatedOrientation);
        // return new Pose3d(this.asMatrix().times(other.asMatrix()));
    }

    public Matrix<N4, N4> asMatrix() {
        double q0 = orientation.w;
        double q1 = orientation.x;
        double q2 = orientation.y;
        double q3 = orientation.z;
        double r00 = 2.0 * (q0 * q0 + q1 * q1) - 1.0;
        double r01 = 2.0 * (q1 * q2 - q0 * q3);
        double r02 = 2.0 * (q1 * q3 + q0 * q2);
        double r10 = 2.0 * (q1 * q2 + q0 * q3);
        double r11 = 2.0 * (q0 * q0 + q2 * q2) - 1.0;
        double r12 = 2.0 * (q2 * q3 - q0 * q1);
        double r20 = 2.0 * (q1 * q3 - q0 * q2);
        double r21 = 2.0 * (q2 * q3 + q0 * q1);
        double r22 = 2.0 * (q0 * q0 + q3 * q3) - 1.0;
        return Matrix.mat(Nat.N4(), Nat.N4()).fill(
            r00, r01, r02, position.x,
            r10, r11, r12, position.y,
            r20, r21, r22, position.z,
            0.0, 0.0, 0.0, 1.0
        );
    }
}
