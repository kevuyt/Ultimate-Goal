package Library4997.MasqControlSystems.MasqPurePursuit;

import java.util.List;

/**
 * Created by Archishmaan Peyyety on 8/11/18.
 * Project: MasqLib
 */

public class MasqPath {
    private double lookAheadDistance = 5;
    private MasqVector carrot, rabbit;
    private MasqPositionTracker positionTracker;
    private List<MasqVector> wayPoints;
    private MasqVector currentWayPoint;
    public MasqPath(List<MasqVector> wayPoints, MasqPositionTracker positionTracker) {
        this.wayPoints = wayPoints;
        this.positionTracker = positionTracker;
        rabbit = new MasqVector(this.positionTracker.getGlobalX(), this.positionTracker.getGlobalY());
        wayPoints.add(0, rabbit);
        currentWayPoint = wayPoints.get(0);
    }
    private void getOrientationError () {
        return;
    }
}
