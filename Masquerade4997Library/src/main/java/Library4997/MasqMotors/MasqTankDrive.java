package Library4997.MasqMotors;

import com.qualcomm.robotcore.hardware.DcMotor;

import Library4997.MasqHardware;
import Library4997.PID_Constants;

/**
 * Created by Archish on 10/28/16.
 */
public class MasqTankDrive implements PID_Constants, MasqHardware {
    private MasqMotor motor1 , motor2, motor3, motor4 = null;
    public MasqTankDrive(String name1, String name2, String name3, String name4) {
        motor1 = new MasqMotor(name1, DcMotor.Direction.REVERSE);
        motor2 = new MasqMotor(name2, DcMotor.Direction.REVERSE);
        motor3 = new MasqMotor(name3, DcMotor.Direction.FORWARD);
        motor4 = new MasqMotor(name4, DcMotor.Direction.FORWARD);
    }
    public void resetEncoders () {
        motor1.resetEncoder();
        motor2.resetEncoder();
        motor3.resetEncoder();
        motor4.resetEncoder();
    }
    public void setPower (double leftPower, double rightPower) {
        motor1.setPower(leftPower);
        motor2.setPower(leftPower);
        motor3.setPower(rightPower);
        motor4.setPower(rightPower);
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
    public void stopDriving() {
        setPower(0,0);
    }
    public void setBrakeMode () {
        motor1.setBrakeMode();
        motor2.setBrakeMode();
        motor3.setBrakeMode();
        motor4.setBrakeMode();
    }
    public boolean isBusy() {
        return (motor1.isBusy() && motor2.isBusy() && motor3.isBusy() && motor4.isBusy());
    }
    public boolean rightIsBusy(){
         return (motor3.isBusy() && motor4.isBusy());
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
    public String getName() {
        return "DRIVETRAIN";
    }
    public String[] getDash() {
        return new String[]{ "Current Position"+ getCurrentPos()};
    }
}