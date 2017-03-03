package BasicLib4997.MasqMotors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;

import BasicLib4997.MasqMotors.MasqRobot.Constants;
import BasicLib4997.MasqMotors.MasqRobot.Direction;
import BasicLib4997.MasqMotors.MasqRobot.MasqRobot;
import BasicLib4997.MasqSensors.MasqClock;

/**
 * Created by Archish on 10/28/16.
 */
public class MasqTankDriveV2 implements Constants {
    private MasqMotorV2 motor1 , motor2, motor3, motor4 = null;
    private double destination = 0;
    private double currentPosition = 0, zeroEncoderPosition= 0;
    private MasqClock clock = new MasqClock();
    public MasqTankDriveV2(String name1, String name2, String name3, String name4) {
        motor1 = new MasqMotorV2(name1, DcMotor.Direction.REVERSE);
        motor2 = new MasqMotorV2(name2, DcMotor.Direction.REVERSE);
        motor3 = new MasqMotorV2(name3, DcMotor.Direction.FORWARD);
        motor4 = new MasqMotorV2(name4, DcMotor.Direction.FORWARD);
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
    public void runUsingEncoder() {
        motor1.runUsingEncoder();
        motor2.runUsingEncoder();
        motor3.runUsingEncoder();
        motor4.runUsingEncoder();
    }
    private boolean opModeIsActive() {
        return ((LinearOpMode) (FtcOpModeRegister.opModeManager.getActiveOpMode())).opModeIsActive();
    }
    public void setDistance (double distance) {
        destination = currentPosition + zeroEncoderPosition;
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


}