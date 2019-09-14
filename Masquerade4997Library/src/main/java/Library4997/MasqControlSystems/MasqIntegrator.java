package Library4997.MasqControlSystems;

/**
 * Created by Archishmaan Peyyety on 9/17/18.
 * Project: MasqLib
 */

public class MasqIntegrator {
    private double prev = 0;
    private double prevTime = 0;
    public double getIntegral (double current) {
        double integral;
        double tChange = System.nanoTime() - prevTime;
        integral = tChange * (prev + (0.5 * (current - prev)));
        prev = current;
        prevTime = System.nanoTime();
        return integral;
    }
}
