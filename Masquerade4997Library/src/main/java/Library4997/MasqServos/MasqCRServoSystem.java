package Library4997.MasqServos;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

import Library4997.MasqUtilities.MasqHelpers.MasqHardware;
import Library4997.MasqSensors.MasqLimitSwitch;

/**
 * Created by Archish on 10/9/17.
 */

public class MasqCRServoSystem implements MasqHardware {
    private final MasqCRServo servo1, servo2, servo3, servo4;
    private final List<MasqCRServo> servos;

    public MasqCRServoSystem(String name1, String name2, HardwareMap hardwareMap) {
        this( new MasqCRServo(name1, hardwareMap), new MasqCRServo(name2, hardwareMap) );
    }

    public MasqCRServoSystem(String name1, String name2, String name3, HardwareMap hardwareMap) {
        this( new MasqCRServo(name1, hardwareMap), new MasqCRServo(name2, hardwareMap), new MasqCRServo(name3, hardwareMap) );
    }

    public MasqCRServoSystem(String name1, String name2, String name3, String name4, HardwareMap hardwareMap) {
        this( new MasqCRServo(name1, hardwareMap), new MasqCRServo(name2, hardwareMap), new MasqCRServo(name3, hardwareMap), new MasqCRServo(name4, hardwareMap) );
    }

    public MasqCRServoSystem(String name1, CRServo.Direction direction1,
                             String name2, CRServo.Direction direction2, HardwareMap hardwareMap) {
        this( new MasqCRServo(name1, direction1, hardwareMap),
                new MasqCRServo(name2, direction2, hardwareMap) );
    }

    public MasqCRServoSystem(String name1, CRServo.Direction direction1,
                             String name2, CRServo.Direction direction2,
                             String name3, CRServo.Direction direction3, HardwareMap hardwareMap) {
        this( new MasqCRServo(name1, direction1, hardwareMap),
                new MasqCRServo(name2, direction2, hardwareMap),
                new MasqCRServo(name3, direction3, hardwareMap) );
    }

    public MasqCRServoSystem(String name1, CRServo.Direction direction1,
                             String name2, CRServo.Direction direction2,
                             String name3, CRServo.Direction direction3,
                             String name4, CRServo.Direction direction4, HardwareMap hardwareMap) {
        this( new MasqCRServo(name1, direction1, hardwareMap),
                new MasqCRServo(name2, direction2, hardwareMap),
                new MasqCRServo(name3, direction3, hardwareMap),
                new MasqCRServo(name4, direction4, hardwareMap) );
    }

    public MasqCRServoSystem(MasqCRServo one, MasqCRServo two) {
        servo1 = one; servo2 = two; servo3 = null; servo4 = null;
        servos = Arrays.asList(servo1, servo2);
    }

    public MasqCRServoSystem(MasqCRServo one, MasqCRServo two, MasqCRServo three) {
        servo1 = one; servo2 = two; servo3 = three; servo4 = null;
        servos = Arrays.asList(servo1, servo2, servo3);
    }

    public MasqCRServoSystem(MasqCRServo one, MasqCRServo two, MasqCRServo three, MasqCRServo four) {
        servo1 = one; servo2 = two; servo3 = three; servo4 = four;
        servos = Arrays.asList(servo1, servo2, servo3, servo4);
    }


    public MasqCRServoSystem setLimits(MasqLimitSwitch min, MasqLimitSwitch max) {
        for (MasqCRServo s : servos) s.setLimits(min, max);
        return this;
    }
    public MasqCRServoSystem setLimit(MasqLimitSwitch min) {
        for (MasqCRServo s: servos) s.setLimit(min);
        return this;
    }

    public void setPower(double power) {
        for (MasqCRServo s : servos) s.setPower(power);
    }

    public double getPower() {return servo1.getPower();}

    @Override
    public String getName() {
        return "SYSTEM";
    }

    @Override
    public String[] getDash() {
        return new String[0];
    }
}
