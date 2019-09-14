package Library4997.MasqServos;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqSensors.MasqLimitSwitch;

/**
 * Created by Archish on 10/28/16.
 */

public class MasqServo implements MasqHardware{
    private Servo servo;
    private String nameServo;
    MasqClock clock = new MasqClock();
    private double targetPosition;
    private double max = 1, min = 0;
    private MasqLimitSwitch limMin, limMax;
    private boolean limDetection;
    public MasqServo(String name, HardwareMap hardwareMap) {
        this.nameServo = name;
        servo = hardwareMap.servo.get(name);
    }
    public MasqServo(String name, Servo.Direction direction, HardwareMap hardwareMap){
        this.nameServo = name;
        servo = hardwareMap.servo.get(name);
        servo.setDirection(direction);
    }
    public void setPosition (double position) {
        targetPosition = position;
        position = ((max - min) * position) + min;
        servo.setPosition(position);
    }
    public void setLimits (MasqLimitSwitch min, MasqLimitSwitch max){
        limMin = min; limMax = max;
        limDetection = true;
    }
    private boolean limitPressed () {
        if (limDetection) return  limMin.isPressed() || limMax.isPressed();
        else return false;
    }
    public double getPosition () {
        return servo.getPosition();
    }
    public void setMax(double max){this.max = max;}
    public void setMin(double min){this.min = min;}
    public void scaleRange (double min, double max) {servo.scaleRange(min,max);}
    public void sleep (int time) throws InterruptedException {
        servo.wait(time);
    }
    public boolean isStalled(int time) {
        boolean isStalled = false;
        double prePos = servo.getPosition();
        if ((servo.getPosition() == prePos && servo.getPosition() != targetPosition)
                && !clock.elapsedTime(time, MasqClock.Resolution.SECONDS)) isStalled = true;
        return isStalled;
    }
    public String getName() {
        return nameServo;
    }

    public String[] getDash() {
        return new String[]{
                "Current Position:" + Double.toString(servo.getPosition()),
                "Stalled:" + Boolean.toString(isStalled(1))
        };
    }
}


