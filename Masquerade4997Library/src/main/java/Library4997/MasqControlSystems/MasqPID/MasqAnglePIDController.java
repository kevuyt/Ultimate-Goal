package Library4997.MasqControlSystems.MasqPID;

import com.qualcomm.robotcore.util.Range;

import Library4997.MasqControlSystems.MasqIntegrator;
import Library4997.MasqSensors.MasqClock;

/**
 * Created by Keval Kataria on 11/11/2019
 */
public class MasqAnglePIDController {
    private MasqIntegrator integrator = new MasqIntegrator();
    private double kp, ki = 0, kd = 0, prevError = 0, prevD = 0;
    private double deriv, timeChange = 0;
    private double error, current, target;
    private MasqClock clock = new MasqClock();

    public MasqAnglePIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }
    public MasqAnglePIDController(double kp, double ki) {
        this.kp = kp;
        this.ki = ki;
    }
    public MasqAnglePIDController(double kp) {
        this.kp = kp;
    }
    public MasqAnglePIDController(MasqPIDConstants constants) {
        kp = constants.kp;
        ki = constants.ki;
        kd = constants.kd;
    }

    //For testing
    public double getOutput (double error, double timeChange) {
        this.timeChange = timeChange;
        clock.reset();
        deriv = (error - prevError) / timeChange;
        prevError = error;
        prevD = deriv;
        return Range.clip((error * kp) +
                (ki * integrator.getIntegral(error, timeChange)) +
                (kd * deriv), -1, 1);
    }

    //For normal use
    public double getOutput (double error) {
        return getOutput(error,clock.milliseconds()/1e3);
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
