package Library4997.MasqUtilities;

import com.qualcomm.robotcore.util.Range;

import Library4997.MasqSensors.MasqClock;

/**
 * Created by Archish on 4/9/18.
 */

public class MasqPIDController {
    private double kp = 0, ki = 0, kd = 0, prevError = 0, prevD = 0;
    private double integral, deriv, timeChange = 0;
    private double error, current, target;
    private MasqClock clock = new MasqClock();

    public MasqPIDController(int kp, int ki, int kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }
    public double getOutput (double current, double target) {
        this.target = target;
        this.current = current;
        error = this.target - this.current;
        timeChange = clock.milliseconds();
        clock.reset();
        integral += error * timeChange;
        deriv = (error - prevError) / timeChange;
        if (deriv < .001) deriv = prevD;
        prevError = error;
        prevD = deriv;
        return Range.clip((error * kp) + (ki * integral) + (kd * deriv), -1, 1);
    }

    public double getKp() {
        return kp;
    }

    public double getKi() {
        return ki;
    }

    public double getKd() {
        return kd;
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

    public double getError() {
        return error;
    }

    public double getCurrent() {
        return current;
    }

    public double getTarget() {
        return target;
    }
}
