package BasicLib4997.MasqMotors;

import com.qualcomm.robotcore.hardware.DcMotor;

import BasicLib4997.MasqHardware;
import BasicLib4997.PID_Constants;

/**
 * Created by Archish on 10/28/16.
 */
public class NeutDriveTrain implements PID_Constants, MasqHardware {
    private MasqMotor motor1 , motor2;
    public NeutDriveTrain(String name1, String name2) {
        motor1 = new MasqMotor(name1, DcMotor.Direction.REVERSE);
        motor2 = new MasqMotor(name2, DcMotor.Direction.FORWARD);
    }
    public void resetEncoders () {
        motor1.resetEncoder();
        motor2.resetEncoder();

    }
    public void setPower (double power) {
        motor1.setPower(power);
        motor2.setPower(power);

    }
    public void setPowerLeft (double power) {
        motor1.setPower(power);
    }
    public void setPowerRight (double power) {
        motor2.setPower(power);
    }
    public void setDistance(int distance){
        motor1.setDistance(distance);
        motor2.setDistance(distance);
    }
    public void runUsingEncoder() {
        motor1.runUsingEncoder();
        motor2.runUsingEncoder();
    }
    public void runToPosition(){
        motor1.runToPosition();
        motor2.runToPosition();
    }
    public void runWithoutEncoders() {
        motor1.runWithoutEncoders();
        motor2.runWithoutEncoders();
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
        return motor1.isBusy() && motor2.isBusy();
    }
    public boolean rightIsBusy(){
        return motor2.isBusy();
    }
    public double getCurrentPos () {

        double motor1Pos = motor1.getCurrentPos();
        double motor2Pos = motor2.getCurrentPos();

        double motor12Pos = (motor1Pos + motor2Pos) / 2;
        return motor12Pos;

    }


    public String getName() {
        return "DriveTrain";
    }

    public String getDash() {
        return "CurrentPosition" + Double.toString(getCurrentPos());
    }
}
