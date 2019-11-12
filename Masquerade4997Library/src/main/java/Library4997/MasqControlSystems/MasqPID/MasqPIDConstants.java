package Library4997.MasqControlSystems.MasqPID;

/**
 * Created by Keval Kataria on 10/11/2019
 */
public class MasqPIDConstants {
    public double kp, ki, kd;

    public MasqPIDConstants(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public void setKp(double kp) {
        this.kp = kp;
    }

    public void setKi(double ki) {
        this.ki = ki;
    }

    public void setKd(double kd) {
        this.kd = kd;
    }
}