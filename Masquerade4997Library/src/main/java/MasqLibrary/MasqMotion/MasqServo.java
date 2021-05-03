package MasqLibrary.MasqMotion;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Servo.Direction;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

import MasqLibrary.MasqSensors.MasqTouchSensor;

import static MasqLibrary.MasqResources.MasqUtils.getHardwareMap;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static java.util.Locale.US;

/**
 * Created by Keval Kataria on 3/15/2021
 */

public class MasqServo {
    ServoImplEx servo;
    private final String name;
    private MasqTouchSensor limMin, limMax;
    private boolean limDetection;
    private boolean prevState = false, taskState = false;

    public MasqServo(String name) {
        this.name = name;
        Servo servo = getHardwareMap().servo.get(name);
        this.servo = new ServoImplEx((ServoControllerEx) servo.getController(), servo.getPortNumber(), ServoConfigurationType.getStandardServoType());
    }
    public MasqServo(String name, Direction direction){
        this.name = name;
        Servo servo = getHardwareMap().servo.get(name);
        this.servo = new ServoImplEx((ServoControllerEx) servo.getController(), servo.getPortNumber(), ServoConfigurationType.getStandardServoType());
        setDirection(direction);
    }

    public void setPosition (double position) {servo.setPosition(position);}
    public void setDirection(Servo.Direction direction) {servo.setDirection(direction);}
    public double getPosition () {
        return servo.getPosition();
    }
    public void scaleRange (double min, double max) {servo.scaleRange(min, max);}

    public void toggle(boolean button, double pos1, double pos2) {
        boolean currState = false;

        if (button) currState = true;
        else if (prevState) taskState = !taskState;

        prevState = currState;

        if (taskState) setPosition(pos1);
        else setPosition(pos2);
    }
    public void toggle (boolean button) {toggle(button, 0, 1);}

    public void switchToggle() {taskState = !taskState;}

    public void disable() {servo.setPwmDisable();}

    @NonNull
    @Override
    public String toString() {
        return String.format(US, "%s:\nPosition: %.2f", name, getPosition());
    }
    public String getName() {return name;}

}