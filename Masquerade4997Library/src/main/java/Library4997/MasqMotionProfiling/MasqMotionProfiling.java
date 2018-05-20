package Library4997.MasqMotionProfiling;

import Library4997.MasqSensors.MasqClock;
import SubSystems4997.MasqRobot;

/**
 * Created by Archish on 5/10/18.
 */

public class MasqMotionProfiling {
    double prevOutput = 0;
    double currentX = 0, currentY = 0, currentTime = 0;
    double increment = .01;
    private MasqClock clock;
    public MasqMotionProfiling () {
        clock = new MasqClock();
    }
    MasqFunction function = new MasqFunction() {
        @Override
        public double getY(double x) {
            return Math.log(x);
        }
    };
    public double getX () {
        updateSystem();
        return currentX;
    }
    public double getY () {
        updateSystem();
        return currentY;
    }
    public void updateSystem () {
        currentX = MasqRobot.positionTracker.getX();
        currentY = MasqRobot.positionTracker.getY();
        currentTime = clock.nanoseconds();
    }
    public void setFunction(MasqFunction function) {
        this.function = function;
    }
}
