package Library4997.MasqServos;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqSensors.MasqLimitSwitch;

/**
 * Created by Archish on 10/10/17.
 */

public class MasqServoSystem implements MasqHardware {
    public final MasqServo servo1, servo2, servo3, servo4;
    private int offset;
    private final List<MasqServo> servos;
    private double position;
    public MasqServoSystem(String name1, String name2, HardwareMap hardwareMap) {
        this(new MasqServo(name1, hardwareMap), new MasqServo(name2, hardwareMap));
    }

    public MasqServoSystem(String name1, String name2, String name3, HardwareMap hardwareMap) {
        this(new MasqServo(name1, hardwareMap), new MasqServo(name2, hardwareMap), new MasqServo(name3, hardwareMap));
    }

    public MasqServoSystem(String name1, String name2, String name3, String name4, HardwareMap hardwareMap) {
        this(new MasqServo(name1, hardwareMap), new MasqServo(name2, hardwareMap), new MasqServo(name3, hardwareMap), new MasqServo(name4, hardwareMap));
    }

    public MasqServoSystem(String name1, Servo.Direction direction1,
                             String name2, Servo.Direction direction2, HardwareMap hardwareMap) {
        this(new MasqServo(name1, direction1, hardwareMap),
                new MasqServo(name2, direction2, hardwareMap));
    }

    public MasqServoSystem(String name1, Servo.Direction direction1,
                             String name2, Servo.Direction direction2,
                             String name3, Servo.Direction direction3, HardwareMap hardwareMap) {
        this( new MasqServo(name1, direction1, hardwareMap),
                new MasqServo(name2, direction2, hardwareMap),
                new MasqServo(name3, direction3, hardwareMap) );
    }

    public MasqServoSystem(String name1, Servo.Direction direction1,
                             String name2, Servo.Direction direction2,
                             String name3, Servo.Direction direction3,
                             String name4, Servo.Direction direction4, HardwareMap hardwareMap) {
        this(new MasqServo(name1, direction1, hardwareMap),
                new MasqServo(name2, direction2, hardwareMap),
                new MasqServo(name3, direction3, hardwareMap),
                new MasqServo(name4, direction4, hardwareMap));
    }

    public MasqServoSystem(MasqServo one, MasqServo two) {
        servo1 = one; servo2 = two; servo3 = null; servo4 = null;
        servos = Arrays.asList(servo1, servo2);
    }

    public MasqServoSystem(MasqServo one, MasqServo two, MasqServo three) {
        servo1 = one; servo2 = two; servo3 = three; servo4 = null;
        servos = Arrays.asList(servo1, servo2, servo3);
    }

    public MasqServoSystem(MasqServo one, MasqServo two, MasqServo three, MasqServo four) {
        servo1 = one; servo2 = two; servo3 = three; servo4 = four;
        servos = Arrays.asList(servo1, servo2, servo3, servo4);
    }

    public double getPosition() {
        return position;
    }
    public MasqServoSystem setLimits(MasqLimitSwitch min, MasqLimitSwitch max) {
        for (MasqServo s : servos) s.setLimits(min, max);
        return this;
    }

    public void setOffset(int offset) {
        this.offset = offset;
    }

    public void setPosition(double position) {
        this.position = position;
        int i = 0;
        for (MasqServo s : servos) {
            s.setPosition(position + (i * offset));
        }
    }

    @Override
    public String getName() {
        return "SERVO SYSTEM";
    }

    @Override
    public String[] getDash() {
        return new String[0];
    }
}
