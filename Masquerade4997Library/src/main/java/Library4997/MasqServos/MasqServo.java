package Library4997.MasqServos;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import Library4997.MasqResources.MasqHardware;
import Library4997.MasqSensors.MasqLimitSwitch;

import static Library4997.MasqUtils.opModeIsActive;

/**
 * Created by Archish on 10/28/16.
 */

public class MasqServo implements MasqHardware{
    private Servo servo;
    private String name;
    private double max = 1, min = 0;
    private MasqLimitSwitch limMin, limMax;
    private boolean limDetection;
    private double position;
    private boolean prevState = false, taskState = false, currState = false;
    private boolean positionControlState = false;


    public MasqServo(String name, HardwareMap hardwareMap) {
        this.name = name;
        servo = hardwareMap.servo.get(name);
    }
    public MasqServo(String name, Servo.Direction direction, HardwareMap hardwareMap){
        this.name = name;
        servo = hardwareMap.servo.get(name);
        servo.setDirection(direction);
    }
    public void setPosition (double position) {
        this.position = position;
        servo.setPosition(position);
    }
    public void setDirection(Servo.Direction direction) {
        servo.setDirection(direction);
    }
    public void setLimits (MasqLimitSwitch min, MasqLimitSwitch max){
        limMin = min; limMax = max;
        limDetection = true;
    }
    private boolean limitPressed () {
        if (limDetection) return  limMin.isPressed() || limMax.isPressed();
        return false;
    }
    public double getPosition () {
        return servo.getPosition();
    }
    public void scaleRange (double min, double max) {
        servo.scaleRange(min, max);
    }
    public void sleep (int time) throws InterruptedException {
        servo.wait(time);
    }

    public void toggle(boolean button, double pos1, double pos2) {
        currState = false;

        if (button) currState = true;
        else if (prevState) taskState = !taskState;

        prevState = currState;

        if (taskState) setPosition(pos1);
        else setPosition(pos2);
    }
    public void toggle (boolean button) {toggle(button, 0, 1);}
    public void setPositionControlState(boolean positionControlState) {
        this.positionControlState = positionControlState;
    }

    public void startPositionControl() {
        positionControlState = true;
        Runnable positionControl = () -> {
            while (opModeIsActive() && positionControlState) setPosition(position);
        };
        Thread positionThread = new Thread(positionControl);
        positionThread.start();
    }

    public String getName() {
        return name;
    }

    public String[] getDash() {
        return new String[]{
                "Current Position:" + servo.getPosition()
        };
    }

}