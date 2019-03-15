package Library4997.MasqControlSystems.MasqPurePursuit;

import java.util.Arrays;
import java.util.List;

import Library4997.MasqResources.MasqMath.MasqPoint;
import Library4997.MasqResources.MasqMath.MasqVector;
import Library4997.MasqWrappers.DashBoard;

/**
 * Created by Archishmaan Peyyety on 8/11/18.
 * Project: MasqLib
 */

public class MasqPath {
    private double lookAheadDistance = 5;
    private List<MasqPoint> wayPoints;
    private MasqVector end;
    private MasqVector carrot;
    public MasqPath (List<MasqPoint> wayPoints, double lookAheadDistance) {
        this.lookAheadDistance = lookAheadDistance;
        this.wayPoints = wayPoints;
        end = new MasqVector(wayPoints.get(1).getX() - wayPoints.get(0).getX(),
                wayPoints.get(1).getY() - wayPoints.get(0).getY());
    }
    public MasqPath (double lookAheadDistance, MasqPoint... wayPoints) {
        this.lookAheadDistance = lookAheadDistance;
        this.wayPoints = Arrays.asList(wayPoints);
        end = new MasqVector(wayPoints[1].getX() - wayPoints[0].getX(),
                wayPoints[1].getY() - wayPoints[0].getY());
    }
    public void updateSystem (MasqPoint robot) {
        MasqVector robotVector = robot.toVector();
        MasqVector unitVector = end.unitVector();
        carrot = unitVector.multiply(robotVector.dotProduct(end) + lookAheadDistance);
    }
    public double getOrientationError(double angle, MasqPoint robot) {
        MasqVector directionVector = new MasqVector(Math.sin(Math.toRadians(angle)), Math.cos(Math.toRadians(angle)), "direction");
        MasqVector robotCarrot = new MasqVector(carrot.getX() - robot.getX(),  carrot.getY() - robot.getY(), "robot");
        DashBoard.getDash().create(directionVector);
        DashBoard.getDash().create(robotCarrot);
        return Math.toDegrees(Math.acos((directionVector.dotProduct(robotCarrot)) / robotCarrot.getMagnitude()));
    }
    public void updatePath(MasqPoint start, MasqPoint end) {

    }
    public List<MasqPoint> getWayPoints() {
        return wayPoints;
    }

    public double[] getQuad(double a, double b, double c) {
        double rootOne, rootTwo;
        rootOne = -(b + Math.sqrt(Math.pow(b, 2) - (4 * a * c))) / (2 * a);
        rootTwo = -(b - Math.sqrt(Math.pow(b, 2) - (4 * a * c))) / (2 * a);
        return new double[] {rootOne, rootTwo};
    }
    public MasqVector getGoalPoint () {
        return carrot;
    }
}
