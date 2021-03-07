package Library4997.MasqMath;

import org.apache.commons.math3.analysis.solvers.PolynomialSolver;

import static java.lang.System.nanoTime;

/**
 * Created by Archishmaan Peyyety on 9/17/18.
 * Project: MasqLib
 */

public class MasqIntegrator {
  private double prev;
  private double prevTime = nanoTime();
  private double integral;

  public double getIntegral (double current) {
    double tChange = nanoTime() / 1e9 - prevTime;
    integral += tChange * (0.5 * (current + prev));
    prev = current;
    prevTime = nanoTime() / 1e9;
    return integral;
  }
}