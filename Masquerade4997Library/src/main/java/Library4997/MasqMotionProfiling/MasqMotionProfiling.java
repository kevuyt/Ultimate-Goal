package Library4997.MasqMotionProfiling;

import Library4997.MasqSensors.MasqClock;
import SubSystems4997.MasqRobot;

/**
 * Created by Archish on 5/10/18.
 */

public class MasqMotionProfiling {
    double prevOutput = 0;
    double currentX = 0, yError = 0, currentY = 0, xError = 0, currentTime = 0;
    double increment = .01;
    private MasqClock clock;
    MasqFunction function = new MasqFunction() {
        @Override
        public double getY(double x) {
            return Math.log(x);
        }
        @Override
        public double getX(double y) {
            return Math.pow(Math.E, y);
        }
        @Override
        public double firstDeriv(double x) {
            return 1/x;
        }
        @Override
        public double secondDeiv(double x) {
            return -1/(Math.pow(x, 2));
        }
    };
    public MasqMotionProfiling () {
        clock = new MasqClock();
    }
    public double getX () {
        updateSystem();
        return currentX;
    }
    public double getY () {
        updateSystem();
        return currentY;
    }
    public void updateSystem () {
        currentX = MasqRobot.positionTracker.getXInches();
        currentY = MasqRobot.positionTracker.getYInches();
        currentTime = clock.nanoseconds();
        xError = currentX - function.getX(currentY);
    }
    public void setFunction(MasqFunction function) {
        this.function = function;
    }
}
