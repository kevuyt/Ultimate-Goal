package BasicLib4997.Servos;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import BasicLib4997.Motors.TankDrive.TankDrive;

/**
 * Created by Archish on 11/4/16.
 */

public class CR_Servo {
    private com.qualcomm.robotcore.hardware.CRServo servo;
    private String nameCr_Servo;
    public CR_Servo(String name){
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
        TankDrive.getTelemetry().addTelemetry(nameCr_Servo + "telemetry");
        TankDrive.getTelemetry().addTelemetry("Current Power:", servo.getPower());
    }
}