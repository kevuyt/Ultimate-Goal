package MasqLibrary.MasqMotion;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;

import MasqLibrary.MasqSensors.MasqTouchSensor;

import static MasqLibrary.MasqResources.MasqUtils.getHardwareMap;
import static java.util.Locale.US;

/**
 * Created by Archish on 11/4/16.
 */

public class MasqCRServo {
    private CRServo servo;
    private String nameCr_Servo;
    private MasqTouchSensor min, max = null;
    private boolean limitDetection;
    public MasqCRServo(String name){
        this.nameCr_Servo = name;
        servo = getHardwareMap().crservo.get(name);
        limitDetection = false;
    }
    public MasqCRServo (String name, CRServo.Direction direction){
        this.nameCr_Servo = name;
        servo = getHardwareMap().crservo.get(name);
        servo.setDirection(direction);
        limitDetection = false;
    }
    public void setLimits(MasqTouchSensor min, MasqTouchSensor max){
        this.min = min; this.max = max;
        limitDetection = true;
    }
    public void setLimit(MasqTouchSensor min) {
        this.min = min;
        this.max = null;
        limitDetection = false;
    }
    public void setPower (double power) {
        double motorPower = power;
        if (limitDetection) {
            if (min != null && min.isPressed() && power < 0 ||
                    max != null && max.isPressed() && power > 0)
                motorPower = 0;
        }
        servo.setPower(motorPower);
    }
    public void sleep (int time) throws InterruptedException {servo.wait(time);}
    public double getPower() {return servo.getPower();}

    @NonNull
    @Override
    public String toString() {
        return String.format(US, "%s:\nPower: %.2f", nameCr_Servo, getPower());
    }
}