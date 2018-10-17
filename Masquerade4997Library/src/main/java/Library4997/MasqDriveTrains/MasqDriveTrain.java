package Library4997.MasqDriveTrains;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqMotors.MasqMotorSystem;
import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqResources.MasqUtils;
import Library4997.MasqSensors.MasqEncoder;


public class MasqDriveTrain implements MasqHardware {
    public MasqMotorSystem leftDrive, rightDrive = null;
    public MasqDriveTrain(String name1, String name2, String name3, String name4, HardwareMap hardwareMap) {
        leftDrive = new MasqMotorSystem(name1, DcMotor.Direction.REVERSE, name2, DcMotor.Direction.REVERSE, "LEFTDRIVE", hardwareMap, MasqMotorModel.REVHDHEX);
        rightDrive = new MasqMotorSystem(name3, DcMotor.Direction.FORWARD, name4, DcMotor.Direction.FORWARD, "RIGHTDRIVE", hardwareMap, MasqMotorModel.REVHDHEX);
    }
    public MasqDriveTrain(HardwareMap hardwareMap){
        leftDrive = new MasqMotorSystem("leftFront", DcMotor.Direction.REVERSE, "leftBack", DcMotor.Direction.REVERSE, "LEFTDRIVE", hardwareMap, MasqMotorModel.REVHDHEX);
        rightDrive = new MasqMotorSystem("rightFront", DcMotor.Direction.FORWARD, "rightBack", DcMotor.Direction.FORWARD, "RIGHTDRIVE", hardwareMap, MasqMotorModel.REVHDHEX);
    }
    public MasqDriveTrain(HardwareMap hardwareMap, MasqMotorModel motorModel){
        leftDrive = new MasqMotorSystem("leftFront", DcMotor.Direction.REVERSE, "leftBack", DcMotor.Direction.REVERSE, "LEFTDRIVE", hardwareMap, motorModel);
        rightDrive = new MasqMotorSystem("rightFront", DcMotor.Direction.FORWARD, "rightBack", DcMotor.Direction.FORWARD, "RIGHTDRIVE", hardwareMap, motorModel);
    }


    public void resetEncoders () {
        leftDrive.resetEncoders();
        rightDrive.resetEncoders();
    }
    public void setPower (double leftPower, double rightPower) {
        leftDrive.setVelocity(leftPower);
        rightDrive.setVelocity(rightPower);
    }
    public void setPower(double power){
        leftDrive.setVelocity(power);
        rightDrive.setVelocity(power);
    }


    public double getRate() {
        return (leftDrive.getVelocity() + rightDrive.getVelocity())/2;
    }
    public double getPower() {
        return (leftDrive.getPower() + rightDrive.getPower()) /2;
    }
    public void setPowerLeft (double power) {
        leftDrive.setVelocity(power);
    }
    public void setPowerRight (double power) {
        rightDrive.setVelocity(power);
    }
    public void runUsingEncoder() {
        leftDrive.runUsingEncoder();
        rightDrive.runUsingEncoder();
    }
    public void runWithoutEncoders() {
        leftDrive.runUsingEncoder();
        rightDrive.runUsingEncoder();
    }
    public void stopDriving() {
        setPower(0,0);
    }
    private boolean opModeIsActive() {
        return MasqUtils.opModeIsActive();
    }
    public void zeroPowerBehavior(){
        rightDrive.breakMotors();
    }
    public double getCurrentPosition() {
        return (leftDrive.getCurrentPosition() + rightDrive.getCurrentPosition())/2;
    }
    public double getAbsolutePositon () {
        return (leftDrive.getAbsolutePosition() + rightDrive.getAbsolutePosition())/2;
    }

    public void setKp(double kp){
        leftDrive.setKp(kp);
        rightDrive.setKp(kp);
    }
    public void setKi(double ki) {
        leftDrive.setKi(ki);
        rightDrive.setKi(ki);
    }
    public void setKd(double kd) {
        leftDrive.setKd(kd);
        rightDrive.setKd(kd);
    }

    public MasqEncoder getEncoder () {
        return leftDrive.motor1.getEncoder();
    }

    public String getName() {
        return "DRIVETRAIN";
    }
    public String[] getDash() {
        return new String[]{
                "Rate "+ getRate(),
                "Left Position: " + Double.toString(leftDrive.getAbsolutePosition()),
                "Right Position: " + Double.toString(rightDrive.getAbsolutePosition()),
        };
    }
}