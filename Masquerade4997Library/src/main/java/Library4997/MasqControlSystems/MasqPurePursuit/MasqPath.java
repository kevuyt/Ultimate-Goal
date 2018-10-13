package Library4997.MasqControlSystems.MasqPurePursuit;

import java.util.List;

import Library4997.MasqResources.MasqUtils;

/**
 * Created by Archishmaan Peyyety on 8/11/18.
 * Project: MasqLib
 */

public class MasqPath {
    private double lookAheadDistance = 5;
    private List<MasqPoint> wayPoints;
    private MasqPoint start = new MasqPoint(3, 1);
    private MasqPoint end = new MasqPoint(1,3);
    private MasqPoint cp = new MasqPoint(1, 1);
    private MasqPoint goalPoint;
    private MasqPoint prevLocation = new MasqPoint(0, 0);
    private MasqLine goalLine, positionLine;
    private MasqLine perpendicularLine = new MasqLine();
    private MasqLine travelLine = new MasqLine();
    public MasqPath (List<MasqPoint> wayPoints, double lookAheadDistance) {
        this.lookAheadDistance = lookAheadDistance;
        this.wayPoints = wayPoints;
        start = wayPoints.get(1);
        end = wayPoints.get(2);
    }
    public MasqPoint getGoalPoint () {
        return goalPoint;
    }
    public void updateSystem (MasqPoint currentPoint) {
        cp = new MasqPoint(currentPoint.getX(), currentPoint.getY());
        double travelM = (start.getY() - end.getY()) / (start.getX() - end.getX());
        travelLine.setM(travelM);
        perpendicularLine.setM(-1/travelM);
        travelLine.setB(start.getY() - (travelLine.getM() * start.getX()));
        perpendicularLine.setB(cp.getY() - (perpendicularLine.getM() * cp.getX()));
        double intersectionX = (-perpendicularLine.getB() + travelLine.getB()) / (perpendicularLine.getM() - travelLine.getM());
        double intersectionY = perpendicularLine.getY(intersectionX);
        MasqPoint intersectionPoint = new MasqPoint(intersectionX, intersectionY);
        double perpendicularDistance = Math.sqrt(Math.pow(intersectionPoint.getY() - cp.getY(),2) +
                Math.pow(intersectionPoint.getX() - cp.getX(),2));

        double distance = Math.sqrt(Math.pow(lookAheadDistance, 2) - Math.pow(perpendicularDistance, 2));
        System.out.println(distance);
        double a, b, c;
        a = Math.pow(travelLine.getM(), 2) + 1;
        b = -2 * (intersectionPoint.getX() - (travelLine.getB() * travelLine.getM()) + (travelLine.getM() * intersectionPoint.getY()));
        c = (Math.pow(travelLine.getB() - intersectionPoint.getY(), 2) - Math.pow(distance, 2) + Math.pow(intersectionPoint.getX(), 2));
        double rootOne, rootTwo;
        rootOne = MasqUtils.max(getQuad(a, b, c)[0], getQuad(a, b, c)[1]);
        rootTwo = MasqUtils.min(getQuad(a, b, c)[0], getQuad(a, b, c)[1]);
        if (((start.getX() - end.getX())) > 0) goalPoint = new MasqPoint(rootTwo, travelLine.getY(rootTwo));
        else goalPoint = new MasqPoint(rootOne, travelLine.getY(rootOne));
        prevLocation = cp;
    }
    public double getOrientationGoal() {
        goalLine = new MasqLine((goalPoint.getY() - cp.getY()) / (goalPoint.getX() - cp.getX()));
        positionLine = new MasqLine((cp.getY() - prevLocation.getY()) / (cp.getX() - prevLocation.getX()));
        return Math.toDegrees(Math.atan((goalLine.getM() - positionLine.getM()) / (1 + (goalLine.getM() * positionLine.getM()))));
    }
    public void updatePath(MasqPoint start, MasqPoint end) {
        this.start = start;
        this.end = end;
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
}
