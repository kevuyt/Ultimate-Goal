package BasicLib4997.Motors;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;

/**
 * Created by Archish on 12/21/16.
 */

public class LiftSystem {
    private Motor liftMotor1;
    private Motor liftMotor2;
    public LiftSystem (String liftmotor1, String liftmotor2) {
        liftMotor1 = new Motor(liftmotor1);
        liftMotor2 = new Motor(liftmotor2);
    }
    public void moveDown (double power) {
        power *= -1;
        liftMotor1.setPower(power);
        liftMotor2.setPower(-power);
    }
    public void moveUp (double power) {
        liftMotor1.setPower(power);
        liftMotor2.setPower(-power);
    }
    public double getPower () {
        return (liftMotor1.getPower() + liftMotor2.getPower()) / 2;
    }
}
