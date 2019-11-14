package Library4997.MasqResources.MasqMath;

/**
 * Created by Archishmaan Peyyety on 9/17/18.
 * Project: MasqLib
 */

public class MasqIntegrator {
    private double prev = 0;
    private double prevTime = System.nanoTime();
    private double integral = 0;
    public double getIntegral (double current, double tChange) {
        integral += tChange * (0.5 * (current + prev));
        prev = current;
        prevTime = System.nanoTime()/1e9;
        return integral;
    }
    public double getIntegral (double current) {
        return getIntegral(current,System.nanoTime()/1e9 - prevTime);
    }
}
