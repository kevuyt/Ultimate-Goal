package MasqLibrary.MasqMath;

import static java.lang.System.nanoTime;

/**
 * Created by Keval Kataria on 3/15/2021
 */

public class MasqIntegrator {
  private double prev;
  private double prevTime;
  private double integral;

  public double getIntegral (double current) {
    double tChange = nanoTime() / 1e9 - prevTime;
    integral += tChange * (0.5 * (current + prev));
    prev = current;
    prevTime = nanoTime() / 1e9;
    return integral;
  }
  public void reset() {
    prev = 0;
    prevTime = nanoTime() / 1e9;
    integral = 0;
  }
}