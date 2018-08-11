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


    public MasqTracker () {
        if (MasqRobot.positionTracker != null) positionTracker = MasqRobot.positionTracker;
    }


    public double getGlobalX () {
        return 0;
    }

    public double getGlobalY () {
        return 0;
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
