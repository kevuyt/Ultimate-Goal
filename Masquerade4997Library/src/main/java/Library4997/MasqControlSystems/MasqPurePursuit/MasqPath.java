package Library4997.MasqControlSystems.MasqPurePursuit;

import java.util.List;

/**
 * Created by Archishmaan Peyyety on 8/11/18.
 * Project: MasqLib
 */

public class MasqPath {
    private double lookAheadDistance = 5;
    private MasqVector carrot =
            new MasqVector(0, 0, "carrot"),
            rabbit;
    private MasqPositionTracker positionTracker;
    private List<MasqVector> wayPoints;
    private MasqVector currentGoal;
    public MasqPath(List<MasqVector> wayPoints, MasqPositionTracker positionTracker) {
        this.wayPoints = wayPoints;
        this.positionTracker = positionTracker;
        rabbit = wayPoints.get(0);
        currentGoal = wayPoints.get(0);
    }
    private void getOrientationError () {
        carrot = new MasqVector(0, 0);
    }
}
