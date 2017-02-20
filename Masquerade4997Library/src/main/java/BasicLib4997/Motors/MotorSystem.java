package BasicLib4997.Motors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import BasicLib4997.Motors.TankDrive.PID_Constants;
import BasicLib4997.Motors.TankDrive.TankDrive;

import static android.R.attr.borderlessButtonStyle;
import static android.R.attr.name;

/**
 * Created by Archish on 10/28/16.
 */
public class MotorSystem implements PID_Constants {
    private Motor motor1 = null;
    private Motor motor2 = null;
    private Motor motor3 = null;
    private Motor motor4 = null;
    public MotorSystem(String name1, String name2, String name3, String name4) {
        motor1 = new Motor(name1, DcMotor.Direction.REVERSE);
        motor2 = new Motor(name2, DcMotor.Direction.REVERSE);
        motor3 = new Motor(name3, DcMotor.Direction.FORWARD);
        motor4 = new Motor(name4, DcMotor.Direction.FORWARD);
    }
    public void resetEncoders () {
        motor1.resetEncoder();
        motor2.resetEncoder();
        motor3.resetEncoder();
        motor4.resetEncoder();
    }
    public void setPower (double power) {
        motor1.setPower(power);
        motor2.setPower(power);
        motor3.setPower(power);
        motor4.setPower(power);
    }
    public void setPowerLeft (double power) {
        motor1.setPower(power);
        motor2.setPower(power);
    }
    public void setPowerRight (double power) {
        motor3.setPower(power);
        motor4.setPower(power);
    }
    public void setDistance(int distance){
        motor1.setDistance(distance);
        motor2.setDistance(distance);
        motor3.setDistance(distance);
        motor4.setDistance(distance);
    }
    public void runUsingEncoder() {
        motor1.runUsingEncoder();
        motor2.runUsingEncoder();
        motor3.runUsingEncoder();
        motor4.runUsingEncoder();
    }
    public void runToPosition(){
        motor1.runToPosition();
        motor2.runToPosition();
        motor3.runToPosition();
        motor4.runToPosition();
    }
    public void runWithoutEncoders() {
        motor1.runWithoutEncoders();
        motor2.runWithoutEncoders();
        motor3.runWithoutEncoders();
        motor4.runWithoutEncoders();
    }
    public void StopDriving() {
        setPower(0);
    }
    public void setBrakeMode () {
        setPower(0.001);
    }
    public static int convert(int TICKS) {
        return (int) ((TICKS * 35.1070765836) / 3 );
    }
    public double convert(double TICKS) {
        return (TICKS * 35.1070765836);
    }
    boolean isStalled () {
        int i;
        boolean isStalled;
        if (motor1.isStalled()) i = 1;
        else if (motor2.isStalled()) i = 2;
        else if (motor3.isStalled()) i = 3;
        else if (motor4.isStalled()) i = 4;
        else i = 0;
        isStalled = i >= 3;
        return isStalled;
    }
    public boolean isBusy() {
        boolean isBusy;
        if (motor1.isBusy() && motor2.isBusy() && motor3.isBusy() && motor4.isBusy()) {
            isBusy = true;
        }
        else {
            isBusy = false;
        }
        return isBusy;
    }
    public boolean rightIsBusy(){
        boolean isBusy;
        if (motor3.isBusy() && motor4.isBusy()) {
            isBusy = true;
        }
        else {
            isBusy = false;
        }
        return isBusy;
    }
    public double getCurrentPos () {

        double motor1Pos = motor1.getCurrentPos();
        double motor2Pos = motor2.getCurrentPos();
        double motor3Pos = motor3.getCurrentPos();
        double motor4Pos = motor4.getCurrentPos();
        double motor12Pos = (motor1Pos + motor2Pos) / 2;
        double motor34Pos = (motor3Pos + motor4Pos) / 2;

        double currentPos = (motor12Pos + motor34Pos) / 2;

        return currentPos;

    }
    public DcMotorController getControllerLeft() {
        return motor1.getController();
    }
    public DcMotorController getControllerRight() {
        return motor3.getController();
    }
    public void telemetryRun () {
        motor1.telemetryRun(false);
        motor2.telemetryRun(false);
        motor3.telemetryRun(false);
        motor4.telemetryRun(false);
        TankDrive.getTelemetry().addTelemetry("Current Position", getCurrentPos());
    }
}
