package frc.robot.util.coprocessor;

public class BoundingBox {
    double upperRightX = 0.0;
    double upperRightY = 0.0;
    double lowerLeftX = 0.0;
    double lowerLeftY = 0.0;

    public BoundingBox(double upperRightX, double upperRightY, double lowerLeftX, double lowerLeftY,
            double boundaryInflate) {
        this.upperRightX = upperRightX + boundaryInflate;
        this.upperRightY = upperRightY + boundaryInflate;
        this.lowerLeftX = lowerLeftX - boundaryInflate;
        this.lowerLeftY = lowerLeftY - boundaryInflate;
    }

    public boolean isObstacleWithinBounds(PointObstacle obstacle) {
        return this.lowerLeftX <= obstacle.getX() && obstacle.getX() <= this.upperRightX &&
                this.lowerLeftY <= obstacle.getY() && obstacle.getY() <= this.upperRightY;
    }
}
