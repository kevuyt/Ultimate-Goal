package Library4997.MasqControlSystems.MasqPID;

import com.qualcomm.robotcore.util.Range;

import Library4997.MasqResources.MasqMath.MasqIntegrator;
import Library4997.MasqSensors.MasqClock;

/**
 * Created by Archish on 4/9/18.
 */

public class MasqPIDController {
    private MasqIntegrator integrator = new MasqIntegrator();
    private double kp;
    private double ki = 0;
    private double kd = 0;
    private double prevError = 0;
    private double prevD = 0;
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

    public void setConstants(int[] constants) {
        this.kp = constants[0];
        this.ki = constants[1];
        this.kd = constants[2];
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

<<<<<<< HEAD
    public MasqClock getClock() {
        return clock;
    }
=======
    public double getKp() {
        return kp;
    }

    public double getKi() {
        return ki;
    }

    public double getKd() {
        return kd;
    }

>>>>>>> 021bd150b96a828f5abcc7e5234ac2cd2a02705e
}