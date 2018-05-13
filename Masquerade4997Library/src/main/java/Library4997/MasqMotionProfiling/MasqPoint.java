package Library4997.MasqMotionProfiling;

/**
 * Created by Archish on 5/13/18.
 */

public class MasqPoint {
    int x, y, theta;

    public MasqPoint(int x, int y, int theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public void setX(int x) {
        this.x = x;
    }

    public void setY(int y) {
        this.y = y;
    }

    public int getX() {
        return x;
    }

    public int getY() {
        return y;
    }

    public int getTheta() {
        return theta;
    }

    public void setTheta(int theta) {
        this.theta = theta;
    }
}
