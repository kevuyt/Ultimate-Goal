package BasicLib4997.MasqMotors;

/**
 * Created by Archish on 12/21/16.
 */

public class LiftSystem {
    private MasqMotor liftMotor1;
    private MasqMotor liftMotor2;
    public LiftSystem (String liftmotor1, String liftmotor2) {
        liftMotor1 = new MasqMotor(liftmotor1);
        liftMotor2 = new MasqMotor(liftmotor2);
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
