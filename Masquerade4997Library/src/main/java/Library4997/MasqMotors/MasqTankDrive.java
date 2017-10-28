package Library4997.MasqMotors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqExternal.MasqHardware;
import Library4997.MasqRobot;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqExternal.Direction;
import Library4997.MasqExternal.PID_CONSTANTS;

/**
 * Created by Archish on 10/28/16.
 */
public class MasqTankDrive implements PID_CONSTANTS, MasqHardware {
    public MasqMotorSystem leftDrive, rightDrive = null;
    private double destination = 0;
    public MasqTankDrive(String name1, String name2, String name3, String name4, HardwareMap hardwareMap) {
        leftDrive = new MasqMotorSystem(name1, DcMotor.Direction.REVERSE, name2, DcMotor.Direction.REVERSE, "LEFTDRIVE", hardwareMap);
        rightDrive = new MasqMotorSystem(name3, DcMotor.Direction.FORWARD, name4, DcMotor.Direction.FORWARD, "RIGHTDRIVE", hardwareMap);
    }
    public MasqTankDrive(HardwareMap hardwareMap){
        leftDrive = new MasqMotorSystem("leftFront", DcMotor.Direction.FORWARD, "leftBack", DcMotor.Direction.FORWARD, "LEFTDRIVE", hardwareMap);
        rightDrive = new MasqMotorSystem("rightFront", DcMotor.Direction.REVERSE, "rightBack", DcMotor.Direction.REVERSE, "RIGHTDRIVE", hardwareMap);
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
        destination = distance;
    }
    public void runUsingEncoder() {
        leftDrive.runUsingEncoder();
        rightDrive.runUsingEncoder();
    }
    public void setClosedLoop(boolean closedLoop){
        leftDrive.setClosedLoop(closedLoop);
        rightDrive.setClosedLoop(closedLoop);
    }

    public void runToPosition(Direction direction, double speed, double timeOut) {
        MasqClock timeoutTimer = new MasqClock();
        resetEncoders();
        int targetClicks = (int)(destination * CLICKS_PER_INCH);
        int clicksRemaining;
        double inchesRemaining;
        double power;
        do {
            clicksRemaining = (int) (targetClicks - Math.abs(getCurrentPosition()));
            inchesRemaining = clicksRemaining / CLICKS_PER_INCH;
            power = direction.value * speed * inchesRemaining * KP_STRAIGHT;
            setPower(power, -power);
        }
        while (opModeIsActive() && inchesRemaining > 0.5 && !timeoutTimer.elapsedTime(timeOut, MasqClock.Resolution.SECONDS));
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
        return MasqRobot.getInstance(null).opModeIsActive();
    }
    public void zeroPowerBehavior(){
        rightDrive.breakMotors();
    }
    public double getCurrentPosition() {
        return (leftDrive.getCurrentPosition() + rightDrive.getCurrentPosition())/2;
    }
    public String getName() {
        return "DRIVETRAIN";
    }
    public String[] getDash() {
        return new String[]{ "Current Position"+ getCurrentPosition()};
    }
}