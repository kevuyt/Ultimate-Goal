package Library4997.MasqControlSystems.MasqPurePursuit;

import java.util.List;

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
    public void updateSystem (MasqPoint robot) {
        MasqVector robotVector = robot.toVector();
        MasqVector unitVector = end.unitVector();
        carrot = unitVector.multiply(robotVector.dotProduct(end) + lookAheadDistance);
    }
    public double getOrientationError(double angle, MasqPoint robot) {
        MasqVector directionVector = new MasqVector(Math.cos(angle), Math.sin(angle));
        MasqVector robotCarrot = new MasqVector(robot.getX() - carrot.getX(), robot.getY() - carrot.getY());
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
