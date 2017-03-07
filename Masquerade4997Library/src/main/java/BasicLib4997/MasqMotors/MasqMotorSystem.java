package BasicLib4997.MasqMotors;

import BasicLib4997.MasqHardware;
import BasicLib4997.PID_Constants;

/**
 * Created by Archish on 2/27/17.
 */

public class MasqMotorSystem implements MasqHardware, PID_Constants {
    double currentPosition, zeroPos, currentPos, prevPos;
    String name;
    public MasqMotorSystem (String name1, String name2, String name3, String name4){
        MasqMotor motor1 = new MasqMotor(name1);
        MasqMotor motor2 = new MasqMotor(name2);
        MasqMotor motor3 = new MasqMotor(name3);
        MasqMotor motor4 = new MasqMotor(name4);
    }
    public void resetEncoders() {

    }
    public String getName() {
        return name;
    }

    public String getDash() {
        return null;
    }
}
