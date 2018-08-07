package Library4997.MasqServos;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqSensors.MasqLimitSwitch;
import Library4997.MasqUtilities.MasqHelpers.MasqHardware;

/**
 * Created by Archish on 11/4/16.
 */

public class MasqCRServo implements MasqHardware{
    private CRServo servo;
    private String nameCr_Servo;
    private MasqLimitSwitch min, max = null;
    private boolean limitDetection;
    public MasqCRServo(String name, HardwareMap hardwareMap){
        this.nameCr_Servo = name;
        servo = hardwareMap.crservo.get(name);
        limitDetection = false;
    }
    public MasqCRServo (String name, CRServo.Direction direction, HardwareMap hardwareMap){
        this.nameCr_Servo = name;
        servo = hardwareMap.crservo.get(name);
        servo.setDirection(direction);
        limitDetection = false;
    }
    public MasqCRServo setLimits(MasqLimitSwitch min, MasqLimitSwitch max){
        this.min = min; this.max = max;
        limitDetection = true;
        return this;
    }
    public MasqCRServo setLimit(MasqLimitSwitch min) {
        this.min = min;
        this.max = null;
        limitDetection = false;
        return this;
    }
    public void setPower (double power) {
        double motorPower = power;
        if (limitDetection) {
            if (min != null && min.getState() && power < 0 ||
                    max != null && max.getState() && power > 0)
                motorPower = 0;
        }
        servo.setPower(motorPower);
    }
    public void sleep (int time) throws InterruptedException {servo.wait(time);}
    public double getPower() {return servo.getPower();}
    public String getName() {return nameCr_Servo;}

    public String[] getDash() {
        return new String[]{
                "Current Power" + Double.toString(getPower())
        };
    }
}