package Library4997.MasqMotors;

import com.qualcomm.robotcore.hardware.DcMotor;

import Library4997.MasqHardware;
import Library4997.PID_CONSTANTS;

/**
 * Created by Archish on 10/28/16.
 */
public class MasqTankDrive implements PID_CONSTANTS, MasqHardware {
    private MasqMotorSystem leftDrive, rightDrive = null;
    public MasqTankDrive(String name1, String name2, String name3, String name4) {
        leftDrive = new MasqMotorSystem(name1, DcMotor.Direction.REVERSE, name2, DcMotor.Direction.REVERSE, "LEFTDRIVE");
        rightDrive = new MasqMotorSystem(name3, DcMotor.Direction.FORWARD, name4, DcMotor.Direction.FORWARD, "RIGHTDRIVE");
    }
    public MasqTankDrive(){
        leftDrive = new MasqMotorSystem("leftFront", DcMotor.Direction.REVERSE, "leftBack", DcMotor.Direction.REVERSE, "LEFTDRIVE");
        rightDrive = new MasqMotorSystem("rightFront", DcMotor.Direction.FORWARD, "rightBack2", DcMotor.Direction.FORWARD, "RIGHTDRIVE");
    }
    public void resetEncoders () {
        leftDrive.resetEncoder();
        rightDrive.resetEncoder();
    }
    public void setPower (double leftPower, double rightPower) {
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }
    public void setPowerLeft (double power) {
        leftDrive.setPower(power);
    }
    public void setPowerRight (double power) {
        rightDrive.setPower(power);
    }
    public void setDistance(int distance){
        leftDrive.setDistance(distance);
        rightDrive.setDistance(distance);
    }
    public void runUsingEncoder() {
       leftDrive.runUsingEncoder();
       rightDrive.runUsingEncoder();
    }
    public void runToPosition(){
        leftDrive.runToPosition();
        rightDrive.runToPosition();
    }
    public void runWithoutEncoders() {
        leftDrive.runUsingEncoder();
        rightDrive.runUsingEncoder();
    }
    public void stopDriving() {
        setPower(0,0);
    }
    public void setBrakeMode () {
        leftDrive.setBrakeMode();
        rightDrive.setBrakeMode();
    }
    public boolean isBusy() {
        return leftDrive.isBusy() &&  rightDrive.isBusy();
    }
    public boolean rightIsBusy(){
         return (rightDrive.isBusy());
    }
    public double getCurrentPos () {
        return (leftDrive.getCurrentPos() + rightDrive.getCurrentPos())/2;
    }
    public String getName() {
        return "DRIVETRAIN";
    }
    public String[] getDash() {
        return new String[]{ "Current Position"+ getCurrentPos()};
    }
}