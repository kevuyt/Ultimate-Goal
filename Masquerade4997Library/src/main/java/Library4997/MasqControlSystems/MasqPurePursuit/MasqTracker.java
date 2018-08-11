package Library4997.MasqControlSystems.MasqPurePursuit;

import Library4997.MasqControlSystems.MasqPositionTracker;
import SubSystems4997.MasqRobot;

/**
 * Created by Archishmaan Peyyety on 8/9/18.
 * Project: MasqLib
 */

public class MasqTracker {
    private MasqPositionTracker positionTracker;
    private double deltaX, deltaY, deltaGlobalHeading, trackWidth;
    private double globalX, globalY;


    public MasqTracker () {
        if (MasqRobot.positionTracker != null) positionTracker = MasqRobot.positionTracker;
    }


    public double getGlobalX () {
        return globalX;
    }

    public double getGlobalY () {
        return globalY;
    }

    public double getTrackWidth() {
        return trackWidth;
    }

    public void setTrackWidth(double trackWidth) {
        this.trackWidth = trackWidth;
    }

    public void updateSystem () {

    }
}
