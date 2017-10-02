package Library4997.MasqServos;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqHardware;
import Library4997.MasqSensors.MasqLimitSwitch;

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
    public MasqCRServo setLimits(MasqLimitSwitch min, MasqLimitSwitch max){
        this.min = min; this.max = max;
        limitDetection = true;
        return this;
    }
    public void setPower (double power) {
        if (!limitDetection)
            servo.setPower(power);
        else {
            if (!max.isPressed() && !min.isPressed())
                servo.setPower(power);
            else
                servo.setPower(0);
        }
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