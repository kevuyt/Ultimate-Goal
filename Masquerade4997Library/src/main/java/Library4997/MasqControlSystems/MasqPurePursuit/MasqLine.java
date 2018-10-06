package Library4997.MasqControlSystems.MasqPurePursuit;

/**
 * Created by Archishmaan Peyyety on 10/6/18.
 * Project: MasqLib
 */

public class MasqLine {
    private double m, b;
    public MasqLine () {}
    public MasqLine(double m) {
        this.m = m;
        b = 0;
    }
    public MasqLine(double m, double b) {
        this.m = m;
        this.b = b;
    }

    public double getM() {
        return m;
    }

    public void setM(double m) {
        this.m = m;
    }

    public double getB() {
        return b;
    }

    public void setB(double b) {
        this.b = b;
    }
    public double getY(double x) {
        return (m*x) + b;
    }
}
