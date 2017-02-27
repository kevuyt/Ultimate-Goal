package BasicLib4997.MasqServos;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;

import BasicLib4997.MasqMotors.MasqRobot.MasqRobot;

/**
 * Created by Archish on 11/4/16.
 */

public class MasqCRServo {
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
    public void getPower() {
        servo.getPower();
    }
    public void telemetryRun () {
        MasqRobot.getTelemetry().addTelemetry(nameCr_Servo + "telemetry");
        MasqRobot.getTelemetry().addTelemetry("Current Power:", servo.getPower());
    }
}