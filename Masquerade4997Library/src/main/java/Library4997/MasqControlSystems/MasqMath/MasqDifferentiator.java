package Library4997.MasqControlSystems.MasqMath;

import Library4997.MasqSensors.MasqClock;

/**
 * Created by Archishmaan Peyyety on 8/7/18.
 * Project: MasqLib
 */

public class MasqDifferentiator {
    private MasqClock clock = new MasqClock();
    private double prevSample, currentSample;
    public double getDerivitive (double currentSample) {
        this.currentSample = currentSample;
        return (currentSample - prevSample) / clock.milliseconds();
    }
    public void update() {
        prevSample = currentSample;
        clock.reset();
    }
}
