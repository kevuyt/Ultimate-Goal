package Library4997.MasqMotionProfiling;

/**
 * Created by Archish on 5/10/18.
 */

public interface MasqFunction {
    double getY(double x);
    double getX(double y);
    double firstDeriv(double x);
    double secondDeiv(double x);
}
