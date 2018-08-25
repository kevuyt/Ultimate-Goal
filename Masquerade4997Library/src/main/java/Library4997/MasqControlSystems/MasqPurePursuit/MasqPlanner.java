package Library4997.MasqControlSystems.MasqPurePursuit;

import java.util.ArrayList;

/**
 * Created by Archishmaan Peyyety on 8/11/18.
 * Project: MasqLib
 */

public class MasqPlanner {
    private double lookaheadDistance = 5; // In Inches
    private MasqVector carrot, rabbit;
    private MasqPositionTracker positionTracker;
    private ArrayList<MasqVector> wayPoints;
    public MasqPlanner(ArrayList<MasqVector> wayPoints, MasqPositionTracker positionTracker) {
        this.wayPoints = wayPoints;
        this.positionTracker = positionTracker;
        carrot = new MasqVector(0, 0);
    }
    public void getTravelHeading () {
        return;
    }
}
