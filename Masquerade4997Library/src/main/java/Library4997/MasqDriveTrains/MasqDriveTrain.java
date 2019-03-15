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
    public MasqDriveTrain(String name1, String name2, String name3, String name4, HardwareMap hardwareMap, MasqMotorModel masqMotorModel) {
        leftDrive = new MasqMotorSystem(name1, DcMotor.Direction.REVERSE, name2, DcMotor.Direction.REVERSE, "LEFTDRIVE", hardwareMap, masqMotorModel);
        rightDrive = new MasqMotorSystem(name3, DcMotor.Direction.FORWARD, name4, DcMotor.Direction.FORWARD, "RIGHTDRIVE", hardwareMap, masqMotorModel);
    }
    public MasqDriveTrain(HardwareMap hardwareMap){
        leftDrive = new MasqMotorSystem("leftFront", DcMotor.Direction.REVERSE, "leftBack", DcMotor.Direction.REVERSE, "LEFTDRIVE", hardwareMap, MasqMotorModel.REVHDHEX);
        rightDrive = new MasqMotorSystem("rightFront", DcMotor.Direction.FORWARD, "rightBack", DcMotor.Direction.FORWARD, "RIGHTDRIVE", hardwareMap, MasqMotorModel.REVHDHEX);
    }
    public MasqDriveTrain(HardwareMap hardwareMap, MasqMotorModel motorModel){
        //FOLLOW DIRECTIONS OF THIS
        leftDrive = new MasqMotorSystem("leftFront", DcMotor.Direction.FORWARD, "leftBack", DcMotor.Direction.FORWARD, "LEFTDRIVE", hardwareMap, motorModel);
        rightDrive = new MasqMotorSystem("rightFront", DcMotor.Direction.REVERSE, "rightBack", DcMotor.Direction.REVERSE, "RIGHTDRIVE", hardwareMap, motorModel);
    }


    public void resetEncoders () {
        leftDrive.resetEncoders();
        rightDrive.resetEncoders();
    }
    public void setVelocity(double leftPower, double rightPower) {
        leftDrive.setVelocity(leftPower);
        rightDrive.setVelocity(rightPower);
    }
    public void setVelocity(double power){
        leftDrive.setVelocity(power);
        rightDrive.setVelocity(power);
    }

    public void setPower(double power) {
        leftDrive.setPower(power);
        rightDrive.setPower(power);
    }
    public void setPower(double power, double p2) {
        leftDrive.setPower(power);
        rightDrive.setPower(p2);
    }


    public double getRate() {
        return (leftDrive.getVelocity() + rightDrive.getVelocity())/2;
    }
    public double getPower() {
        return (leftDrive.getPower() + rightDrive.getPower()) /2;
    }
    public void setVelocityLeft(double power) {
        leftDrive.setVelocity(power);
    }
    public void setVelocityRight(double power) {
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
        setVelocity(0,0);
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

    public void setClosedLoop (boolean closedLoop) {
        leftDrive.setClosedLoop(closedLoop);
        rightDrive.setClosedLoop(closedLoop);
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