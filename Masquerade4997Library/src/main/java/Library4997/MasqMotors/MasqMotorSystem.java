package Library4997.MasqMotors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqSensors.MasqLimitSwitch;

/**
 * MasqMotorSystem That supports two or more motors and treats them as one
 */
public class MasqMotorSystem implements MasqHardware {
    public MasqMotor motor1 , motor2, motor3;
    public List<MasqMotor> motors;
    public int numMotors;
    double kp, ki, kd;
    private String systemName;
    private MasqMotorModel encoder = MasqMotorModel.ORBITAL20;
    public MasqMotorSystem(String name1, DcMotor.Direction direction, String name2, DcMotor.Direction direction2, String systemName, HardwareMap hardwareMap, MasqMotorModel encoder) {
        this.systemName = systemName;
        motor1 = new MasqMotor(name1, encoder, direction, hardwareMap);
        motor2 = new MasqMotor(name2, encoder, direction2, hardwareMap);
        motor3 = null;
        motors = Arrays.asList(motor1, motor2);
        numMotors = 2;//
    }
    public MasqMotorSystem(String name1, String name2, String systemName, HardwareMap hardwareMap, MasqMotorModel encoder) {
        this.systemName = systemName;
        motor1 = new MasqMotor(name1, encoder, DcMotor.Direction.FORWARD, hardwareMap);
        motor2 = new MasqMotor(name2, encoder, DcMotor.Direction.FORWARD, hardwareMap);
        motor3 = null;
        motors = Arrays.asList(motor1, motor2);
        numMotors = 2;
    }
    public MasqMotorSystem(String name1, DcMotor.Direction direction,
                           String name2, DcMotor.Direction direction2,
                           String name3, DcMotor.Direction direction3, String systemName,
                           HardwareMap hardwareMap, MasqMotorModel encoder) {
        this.systemName = systemName;
        motor1 = new MasqMotor(name1, encoder, direction, hardwareMap);
        motor2 = new MasqMotor(name2, encoder, direction2, hardwareMap);
        motor3 = new MasqMotor(name3, encoder, direction3, hardwareMap);
        motors = Arrays.asList(motor1, motor2, motor3);
        numMotors = 3;
    }
    public MasqMotorSystem(String name1, String name2, String name3, String systemName, HardwareMap hardwareMap, MasqMotorModel encoder) {
        this.systemName = systemName;
        motor1 = new MasqMotor(name1, encoder, DcMotor.Direction.FORWARD, hardwareMap);
        motor2 = new MasqMotor(name2, encoder, DcMotor.Direction.FORWARD, hardwareMap);
        motor3 = new MasqMotor(name3, encoder, DcMotor.Direction.FORWARD, hardwareMap);
        motors = Arrays.asList(motor1, motor2, motor3);
        numMotors = 3;
    }
    public MasqMotorSystem(String name1, String name2, MasqMotorModel encoder, HardwareMap hardwareMap) {
        motor1 = new MasqMotor(name1, encoder, DcMotor.Direction.FORWARD, hardwareMap);
        motor2 = new MasqMotor(name2, encoder, DcMotor.Direction.FORWARD, hardwareMap);
        motors = Arrays.asList(motor1, motor2);
        numMotors = 2;
    }
    public MasqMotorSystem(String name1,DcMotor.Direction d1, String name2, DcMotor.Direction d2, MasqMotorModel encoder, HardwareMap hardwareMap) {
        motor1 = new MasqMotor(name1, encoder, d1, hardwareMap);
        motor2 = new MasqMotor(name2, encoder, d2, hardwareMap);
        motors = Arrays.asList(motor1, motor2);
        numMotors = 2;
    }
    public MasqMotorSystem(String name, MasqMotorModel masqMotorModel, DcMotor.Direction direction, HardwareMap hardwareMap) {
        motor1 = new MasqMotor(name, masqMotorModel, direction, hardwareMap);
        motors = Collections.singletonList(motor1);
        numMotors = 1;
    }
    public void resetEncoders() {
        for (MasqMotor masqMotor : motors)
            masqMotor.resetEncoder();
    }
    public void setKp(double kp){
        this.kp = kp;
        for (MasqMotor masqMotor: motors) masqMotor.setKp(kp);
    }
    public void setKi(double ki){
        this.ki = ki;
        for (MasqMotor masqMotor: motors) masqMotor.setKi(ki);
    }
    public void setKd(double kd){
        this.kd = kd;
        for (MasqMotor masqMotor: motors) masqMotor.setKd(kd);
    }
    public double getPower() {
        double num = 0, sum = 0;
        for (MasqMotor masqMotor: motors) {
            sum += Math.abs(masqMotor.getPower());
            num++;
        }
        return sum/num;
    }
    public double getInches () {
        double num = 0, sum = 0;
        for (MasqMotor masqMotor : motors) {
            sum += masqMotor.getEncoder().getInches();
            num++;
        }
        return sum/num;
    }
    public void setMinPower(double power) {
        for (MasqMotor masqMotor : motors) masqMotor.setMinPower(power);
    }
    public void setVelocity(double power) {
        for (MasqMotor masqMotor : motors) masqMotor.setVelocity(power);
    }
    public void setVelocities(double... powers) {
        for (int i = 0; i < numMotors; i++)  motors.get(i).setVelocity(powers[i]);
    }
    public void setPower(double power) {
        for (MasqMotor masqMotor : motors)masqMotor.setPower(power);
    }
    public void setPowers(double... powers) {
        for (int i = 0; i < numMotors; i++)  motors.get(i).setPower(powers[i]);

    }
    public void setClosedLoop (boolean closedLoop) {
        for (MasqMotor masqMotor : motors) {
            masqMotor.setClosedLoop(closedLoop);
        }
    }
    public void setLimits (MasqLimitSwitch min, MasqLimitSwitch max) {
        for (MasqMotor masqMotor : motors) {
            masqMotor.setLimits(min, max);
        }
    }

    public void runUsingEncoder() {
        for (MasqMotor masqMotor: motors)
            masqMotor.runUsingEncoder();
    }
    public void runWithoutEncoder() {
        for (MasqMotor masqMotor: motors)
            masqMotor.runWithoutEncoder();
    }
    public double getAngle () {
        double sum = 0, num = 0;
        for (MasqMotor masqMotor: motors) {
            sum += masqMotor.getAngle();
            num++;
        }
        return sum/num;
    }
    public double getAveragePositivePosition() {
        double sum = 0;
        double num = 1;
        for (MasqMotor motor : motors) {
            sum += Math.abs(motor.getCurrentPosition());
            num++;
        }
        return sum/num;
    }
    public double getVelocity(){
        double i = 0;
        double rate = 0;
        for (MasqMotor masqMotor: motors){
            rate += masqMotor.getVelocity();
            i++;
        }
        return rate/i;
    }
    public boolean isBusy() {
        for (MasqMotor masqMotor: motors) if(masqMotor.isBusy()) return true;
        return false;
    }
    public int getCurrentPosition() {
        int total = 0;
        for (MasqMotor m : motors) total += m.getCurrentPosition();
        return total / numMotors;
    }
    public int getAbsolutePosition ()  {
        int total = 0;
        for (MasqMotor m : motors) total += m.getAbsolutePosition();
        return total / numMotors;
    }

    public double getKp() {
        return kp;
    }

    public double getKi() {
        return ki;
    }

    public double getKd() {
        return kd;
    }

    public MasqMotorModel getEncoder() {
        return encoder;
    }
    public void setWheelDiameter(double diameter) {
        for (MasqMotor motor : motors) {
            motor.encoder.setWheelDiameter(diameter);
        }
    }

    public void setEncoder(MasqMotorModel encoder) {
        this.encoder = encoder;
    }

    public String getName() {
        return systemName;
    }
    public String[] getDash() {
        return new String[]{ "Current Position" + getCurrentPosition()};
    }
}