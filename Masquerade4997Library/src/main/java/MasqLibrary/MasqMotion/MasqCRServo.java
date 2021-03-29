package MasqLibrary.MasqMotion;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

import MasqLibrary.MasqSensors.MasqTouchSensor;

import static MasqLibrary.MasqResources.MasqUtils.getHardwareMap;
import static java.util.Locale.US;

/**
 * Created by Keval Kataria on 3/15/2021
 */

public class MasqCRServo {
    private CRServoImplEx servo;
    private String name;
    private MasqTouchSensor min, max = null;
    private boolean limitDetection;

    public MasqCRServo(String name){
        this.name = name;
        CRServo servo = getHardwareMap().crservo.get(name);
        this.servo = new CRServoImplEx((ServoControllerEx) servo.getController(), servo.getPortNumber(), ServoConfigurationType.getStandardServoType());
        limitDetection = false;
    }
    public MasqCRServo (String name, Direction direction){
        this(name);
        servo.setDirection(direction);
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
    public double getPower() {return servo.getPower();}

    @NonNull
    @Override
    public String toString() {
        return String.format(US, "%s:\nPower: %.2f", name, getPower());
    }
}