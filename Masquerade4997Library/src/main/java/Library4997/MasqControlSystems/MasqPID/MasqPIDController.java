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
    
    public double[] getConstants() {
        return new double[]{kp, ki, kd};
    }

    public void setConstants(double kp, double ki, double kd) {
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

    public MasqClock getClock() {
        return clock;
    }
}
