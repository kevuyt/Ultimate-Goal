package Library4997.MasqServos;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Servo;
import Library4997.MasqExternal.MasqHardware;
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
    private double max, min;
    private MasqLimitSwitch limMin, limMax;
    private boolean limDetection;
    public MasqServo(String name, HardwareMap hardwareMap){
        this.nameServo = name;
        servo = hardwareMap.servo.get(name);
    }
    public void setPosition (double position) {
        targetPosition = position;
        servo.setPosition(position);
    }
    public void setPositionV2 (double position) {
        targetPosition = position;
        double newPosition = servo.getPosition();
        while (newPosition < position && !isStalled(1) && !limPressed()) {
            servo.setPosition(newPosition);
            newPosition += 0.01;
        }
    }
    public void setLimMin (MasqLimitSwitch min){limMin = min; limDetection = true;}
    public void setLimMax (MasqLimitSwitch max){limMax = max; limDetection = true;}
    private boolean limPressed () {
        if (limDetection) return  limMin.isPressed() || limMax.isPressed();
        else return false;
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


