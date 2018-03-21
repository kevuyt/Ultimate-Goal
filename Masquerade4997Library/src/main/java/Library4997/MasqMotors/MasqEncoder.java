package Library4997.MasqMotors;

/**
 * Created by Archish on 3/14/18.
 */

public class MasqEncoder {
    MasqMotor motor;
    public MasqEncoder (MasqMotor motor) {
        this.motor = motor;
    }
    public double getPosition () {
        return motor.getCurrentPosition();
    }
}
