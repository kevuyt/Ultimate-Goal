package BasicLib4997.MasqMotors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;

import BasicLib4997.PID_Constants;
import BasicLib4997.MasqMotors.MasqRobot.Direction;
import BasicLib4997.MasqSensors.MasqClock;

/**
 * Created by Archish on 10/28/16.
 */
public class MasqTankDriveV2 implements PID_Constants {
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
    public void setPower (double powerLeft, double powerRight) {
        motor1.setPower(powerLeft);
        motor2.setPower(powerLeft);
        motor3.setPower(powerRight);
        motor4.setPower(powerRight);
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
    public double getCurrentPos () {
        return currentPosition;
    }


}