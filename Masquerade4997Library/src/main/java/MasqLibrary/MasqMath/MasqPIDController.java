package MasqLibrary.MasqMath;

import com.qualcomm.robotcore.util.ElapsedTime;

import static com.qualcomm.robotcore.util.Range.clip;

/**
 * Created by Keval Kataria on 3/15/2021
 */

public class MasqPIDController {
    private MasqIntegrator integrator = new MasqIntegrator();
    private double kp = 0;
    private double ki = 0;
    private double kd = 0;
    private double prevError = 0;
    private final ElapsedTime clock = new ElapsedTime();

    public MasqPIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }
    public MasqPIDController(double kp) {this.kp = kp;}
    public MasqPIDController() {}

    public double getOutput (double error) {
        double timeChange = clock.seconds();
        double derivative = (error - prevError) / timeChange;
        double integral = integrator.getIntegral(error);

        clock.reset();
        prevError = error;

        return clip((error * kp) +
                (ki * integral) +
                (kd * derivative), -1, 1);
    }

    public double[] getConstants() {
        return new double[] {kp, ki, kd};
    }

    public void setConstants(double[] constants) {
        this.kp = constants[0];
        this.ki = constants[1];
        this.kd = constants[2];
    }

    public void setKp(double kp) {this.kp = kp;}
    public void setKi(double ki) {this.ki = ki;}
    public void setKd(double kd) {this.kd = kd;}

    public void reset() {
        clock.reset();
        prevError = 0;
        integrator.reset();
    }
}