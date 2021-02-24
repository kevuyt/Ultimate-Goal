package Library4997.MasqDriveTrains;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqMotors.MasqMotorModel;
import Library4997.MasqMotors.MasqMotorSystem;
import Library4997.MasqResources.MasqHardware;
import Library4997.MasqSensors.MasqEncoder;

import static Library4997.MasqMotors.MasqMotorModel.REVHDHEX20;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.*;
import static java.lang.Math.abs;


public class MasqDriveTrain implements MasqHardware {
    public MasqMotorSystem leftDrive, rightDrive;

    public MasqDriveTrain(String name1, String name2, String name3, String name4, HardwareMap hardwareMap) {
        leftDrive = new MasqMotorSystem(name1, REVERSE, name2, REVERSE, "LEFTDRIVE", hardwareMap, REVHDHEX20);
        rightDrive = new MasqMotorSystem(name3, FORWARD, name4, FORWARD, "RIGHTDRIVE", hardwareMap, REVHDHEX20);
    }
    public MasqDriveTrain(String name1, String name2, String name3, String name4, HardwareMap hardwareMap, MasqMotorModel masqMotorModel) {
        leftDrive = new MasqMotorSystem(name1, REVERSE, name2, REVERSE, "LEFTDRIVE", hardwareMap, masqMotorModel);
        rightDrive = new MasqMotorSystem(name3, FORWARD, name4, FORWARD, "RIGHTDRIVE", hardwareMap, masqMotorModel);
    }
    public MasqDriveTrain(HardwareMap hardwareMap){
        leftDrive = new MasqMotorSystem("leftFront", FORWARD, "leftBack", FORWARD, "LEFTDRIVE", hardwareMap, REVHDHEX20);
        rightDrive = new MasqMotorSystem("rightFront", REVERSE, "rightBack", REVERSE, "RIGHTDRIVE", hardwareMap, REVHDHEX20);
    }
    public MasqDriveTrain(HardwareMap hardwareMap, MasqMotorModel motorModel){
        leftDrive = new MasqMotorSystem("leftFront", FORWARD, "leftBack", FORWARD, "LEFTDRIVE", hardwareMap, motorModel);
        rightDrive = new MasqMotorSystem("rightFront", REVERSE, "rightBack", REVERSE, "RIGHTDRIVE", hardwareMap, motorModel);
    }
    public MasqDriveTrain(MasqMotorSystem left, MasqMotorSystem right) {
        leftDrive = left;
        rightDrive = right;
    }

    public void resetEncoders () {
        leftDrive.resetEncoders();
        rightDrive.resetEncoders();
    }

    public void setVelocity(double leftPower, double rightPower) {
        rightDrive.setVelocity(rightPower);
        leftDrive.setVelocity(leftPower);
    }
    public void setVelocity(double power){
        leftDrive.setVelocity(power);
        rightDrive.setVelocity(power);
    }
    public void setVelocity(double leftFront, double leftBack, double rightFront, double rightBack) {
        leftDrive.motor1.setVelocity(leftFront);
        leftDrive.motor2.setVelocity(leftBack);
        rightDrive.motor1.setVelocity(rightFront);
        rightDrive.motor2.setVelocity(rightBack);
    }

    public double getInches() {return (leftDrive.getInches() + rightDrive.getInches())/2;}

    public double getVelocity() {
        return (leftDrive.getVelocity() + rightDrive.getVelocity())/2;
    }
    public double getPower() {
        return (leftDrive.getPower() + rightDrive.getPower()) /2;
    }

    public int getCurrentPosition() {
        return (leftDrive.getCurrentPosition() + rightDrive.getCurrentPosition())/2;
    }
    public double getCurrentPositionPositive() {
        return (abs(leftDrive.motor1.getCurrentPosition()) +
                abs(leftDrive.motor2.getCurrentPosition()) +
                abs(rightDrive.motor1.getCurrentPosition()) +
                abs(rightDrive.motor1.getCurrentPosition()))/4;
    }

    public void setClosedLoop (boolean closedLoop) {
        leftDrive.setClosedLoop(closedLoop);
        rightDrive.setClosedLoop(closedLoop);
    }

    public void setKp(double kp){
        leftDrive.setKp(kp);
        rightDrive.setKp(kp);
    }

    public void startVelocityControl() {
        leftDrive.startVelocityControl();
        rightDrive.startVelocityControl();
    }

    public MasqEncoder getEncoder () {return rightDrive.motor1.getEncoder();}

    public String getName() {return "DRIVETRAIN";}
    public String[] getDash() {
        return new String[]{
                "Rate "+ getVelocity(),
                "Left Position: " + leftDrive.getAbsolutePosition(),
                "Right Position: " + rightDrive.getAbsolutePosition(),
        };
    }
}