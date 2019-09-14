package Library4997.MasqControlSystems.MasqPID;

import com.qualcomm.robotcore.util.Range;

import Library4997.MasqControlSystems.MasqIntegrator;
import Library4997.MasqSensors.MasqClock;

/**
 * Created by Archish on 4/9/18.
 */

public class MasqPIDController {
    private MasqIntegrator integrator = new MasqIntegrator();
    private double kp, ki = 0, kd = 0, prevError = 0, prevD = 0;
    private double deriv, timeChange = 0;
    private double error, current, target;
    private MasqClock clock = new MasqClock();

    public MasqPIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }
    public MasqPIDController(double kp, double ki) {
        this.kp = kp;
        this.ki = ki;
    }
    public MasqPIDController(double kp) {
        this.kp = kp;
    }
    public double getOutput (double current, double target) {
        this.target = target;
        this.current = current;
        error = this.target - this.current;
        timeChange = clock.milliseconds();
        clock.reset();
        deriv = (error - prevError) / timeChange;
        prevError = error;
        prevD = deriv;
        return Range.clip((error * kp) +
                (ki * integrator.getIntegral(error)) +
                (kd * deriv), -1, 1);
    }
    public double getOutput (double error) {
        timeChange = clock.milliseconds();
        clock.reset();
        deriv = (error - prevError) / timeChange;
        prevError = error;
        prevD = deriv;
        return Range.clip((error * kp) +
                (ki * integrator.getIntegral(error)) +
                (kd * deriv), -1, 1);
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

    public double getPrevError() {
        return prevError;
    }

    public double getPrevD() {
        return prevD;
    }

    public double getDeriv() {
        return deriv;
    }

    public double getTimeChange() {
        return timeChange;
    }

    public MasqClock getClock() {
        return clock;
    }
}
