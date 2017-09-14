package Library4997.MasqServos;

import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;

import Library4997.MasqHardware;
import Library4997.MasqSensors.MasqClock;

/**
 * Created by Archish on 10/28/16.
 */

public class MasqServo implements MasqHardware{
    private com.qualcomm.robotcore.hardware.Servo servo;
    private String nameServo;
    private double zero = 0;
    MasqClock clock = new MasqClock();
    public MasqServo(String name){
        this.nameServo = name;
        servo = FtcOpModeRegister.opModeManager.getHardwareMap().servo.get(name);
    }
    public void setPosition (double angle, double maxPos) {
        angle = logicalToPhysical(angle,maxPos);
        servo.setPosition(angle);
    }
    public void setPosition (double position) {
        servo.setPosition(position);
    }
    public void scaleRange (double min, double max) {
        servo.scaleRange(min,max);
    }
    public void sleep (int time) throws InterruptedException {
        servo.wait(time);
    }
    public double logicalToPhysical (double angle, double maxPosition) {
        double convertedNum;
        convertedNum = angle/maxPosition;
        return convertedNum;
    }
    public ServoController getController (){
        return servo.getController();
    }
    public boolean isStalled(int time, double targetPosition) {
        boolean isStalled = false;
        double prePos = servo.getPosition();
        if ((servo.getPosition() == prePos && servo.getPosition() != targetPosition) && !clock.elapsedTime(time, MasqClock.Resolution.SECONDS)) {
            isStalled = true;
        }
        return isStalled;
    }
    public boolean isStalled(double targetPosition) {
        boolean isStalled = false;
        double prePos = servo.getPosition();
        if ((servo.getPosition() == prePos && servo.getPosition() != targetPosition) && !clock.elapsedTime(1, MasqClock.Resolution.SECONDS)) {
            isStalled = true;
        }
        return isStalled;
    }
    public double getZero() {
        return zero;
    }
    public void setZero(double zero) {
        this.zero = zero;
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


