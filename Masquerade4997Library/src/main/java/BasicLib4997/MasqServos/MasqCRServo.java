package BasicLib4997.MasqServos;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;

import BasicLib4997.DashBoard;
import BasicLib4997.MasqHardware;

/**
 * Created by Archish on 11/4/16.
 */

public class MasqCRServo implements MasqHardware{
    private com.qualcomm.robotcore.hardware.CRServo servo;
    private String nameCr_Servo;
    public MasqCRServo(String name){
        this.nameCr_Servo = name;
        servo = FtcOpModeRegister.opModeManager.getHardwareMap().crservo.get(name);
    }
    public void setPower (double power) {
        servo.setPower(power);
    }
    public void sleep (int time) throws InterruptedException {
        servo.wait(time);
    }
    public double getPower() {
        return servo.getPower();
    }
    public void telemetryRun () {
        DashBoard.getDash().create(nameCr_Servo + "telemetry");
        DashBoard.getDash().create("Current Power:", servo.getPower());
    }

    public String getName() {
        return nameCr_Servo;
    }

    public String[] getDash() {
        return new String[]{
                "Current Power" + Double.toString(getPower())
        };
    }
}