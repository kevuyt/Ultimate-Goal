package Library4997.MasqMotors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;

import Library4997.MasqHardware;
import Library4997.MasqWrappers.Direction;
import Library4997.PID_Constants;

/**
 * Created by Archish on 10/28/16.
 */
public class MasqTankDrive implements PID_Constants, MasqHardware {
    private MasqMotorSystem leftDrive, rightDrive = null;
    private double destination = 0;
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
    public void setPower(double power){
        leftDrive.setPower(power);
        rightDrive.setPower(power);
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
    public void runToPosition(Direction direction, double speed){
        resetEncoders();
        int targetClicks = (int)(destination * CLICKS_PER_CM);
        int clicksRemaining;
        double inchesRemaining;
        double power;
        do {
            clicksRemaining = (int) (targetClicks - Math.abs(getCurrentPos()));
            inchesRemaining = clicksRemaining / CLICKS_PER_CM;
            power = direction.value * speed * inchesRemaining * KP_STRAIGHT;
            setPower(power);
        } while (opModeIsActive() && inchesRemaining > 0.5);
        setPower(0);
    }
    public void runWithoutEncoders() {
        leftDrive.runUsingEncoder();
        rightDrive.runUsingEncoder();
    }
    public void stopDriving() {
        setPower(0,0);
    }
    private boolean opModeIsActive() {
        return ((LinearOpMode) (FtcOpModeRegister.opModeManager.getActiveOpMode())).opModeIsActive();
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