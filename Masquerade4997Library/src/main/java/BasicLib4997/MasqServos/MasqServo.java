package BasicLib4997.MasqServos;

import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import BasicLib4997.MasqMotors.TankDrive.MasqRobot;

/**
 * Created by Archish on 10/28/16.
 */

public class MasqServo {
    private Telemetry telemetry;
    private com.qualcomm.robotcore.hardware.Servo servo;
    private String nameServo;
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
    public void telemetryRun () {
        MasqRobot.getTelemetry().addTelemetry(nameServo + "telemetry");
        MasqRobot.getTelemetry().addTelemetry("Current Position:", servo.getPosition());
    }

}


