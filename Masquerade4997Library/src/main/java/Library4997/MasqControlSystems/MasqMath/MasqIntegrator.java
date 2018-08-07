package Library4997.MasqControlSystems.MasqMath;

import Library4997.MasqSensors.MasqClock;

/**
 * Created by Archishmaan Peyyety on 8/7/18.
 * Project: MasqLib
 */

public class MasqIntegrator {
    private double sum = 0;
    private MasqClock clock = new MasqClock();

    public double getIntegral () {
        return sum;
    }
    public void update (double currentSample) {
        sum += currentSample * clock.milliseconds();
        clock.reset();
    }
}
