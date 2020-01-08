package Library4997.MasqSensors.MasqPositionTracker;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Archishmaan Peyyety on 2020-01-08.
 * Project: MasqLib
 */
public class MasqPositionTracker {
    private List<MasqDeadwheel> wheels = new ArrayList<>();
    private double globalX, globalY;
    private int xWheels, yWheels;
    public void updateSystem() {

    }

    public double getGlobalX() {
        return globalX;
    }

    public double getGlobalY() {
        return globalY;
    }

    public void addWheel(MasqDeadwheel deadwheel) {
        if (deadwheel.getMeasurement() == MasqDeadwheel.Measurement.X) xWheels++;
        else yWheels++;
        wheels.add(deadwheel);
    }
}
