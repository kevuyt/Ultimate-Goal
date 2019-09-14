package Library4997.MasqMotors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqSensors.MasqLimitSwitch;

/**
 * MasqMotorSystem That supports two or more motors and treats them as one
 */
public class MasqMotorSystem implements MasqHardware {
    public MasqMotor motor1 , motor2, motor3;
    private List<MasqMotor> motors;
    private int numMotors;
    double kp, ki, kd;
    private double currentPower = 0;
    private double slowDown = 0;
    private String systemName;
    private MasqMotorModel encoder = MasqMotorModel.REVHDHEX;
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
    public MasqMotorSystem(String name1,DcMotor.Direction d, String name2, DcMotor.Direction d1, MasqMotorModel encoder, HardwareMap hardwareMap) {
        motor1 = new MasqMotor(name1, encoder, d, hardwareMap);
        motor2 = new MasqMotor(name2, encoder, d1, hardwareMap);
        motors = Arrays.asList(motor1, motor2);
        numMotors = 2;
    }
    public void setBreakMode() {
        for (MasqMotor masqMotor : motors)
            masqMotor.setBreakMode();
    }
    public MasqMotorSystem resetEncoders() {
        for (MasqMotor masqMotor : motors)
            masqMotor.resetEncoder();
        return this;
    }
    public MasqMotorSystem setKp(double kp){
        this.kp = kp;
        for (MasqMotor masqMotor: motors) masqMotor.setKp(kp);
        return this;
    }
    public MasqMotorSystem setKi(double ki){
        this.ki = ki;
        for (MasqMotor masqMotor: motors) masqMotor.setKi(ki);
        return this;
    }
    public MasqMotorSystem setKd(double kd){
        this.kd = kd;
        for (MasqMotor masqMotor: motors) masqMotor.setKd(kd);
        return this;
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
        currentPower = power;
        for (MasqMotor masqMotor : motors) masqMotor.setVelocity(power);
    }
    public void setPower(double power) {
        for (MasqMotor masqMotor : motors)masqMotor.setPower(power);
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
    public void setAcceleration (double target) {
        for (MasqMotor masqMotor: motors) masqMotor.setAcceleration(target);
    }
    public MasqMotorSystem setDistance(int distance){
        for (MasqMotor masqMotor: motors)
            masqMotor.setDistance(distance);
        return this;
    }
    public MasqMotorSystem runUsingEncoder() {
        for (MasqMotor masqMotor: motors)
            masqMotor.runUsingEncoder();
        return this;
    }
    public MasqMotorSystem breakMotors(){
        for (MasqMotor masqMotor: motors)
            masqMotor.setBreakMode();
        return this;
    }
    public MasqMotorSystem unBreakMotors(){
        for (MasqMotor masqMotor: motors)
            masqMotor.unBreakMode();
        return this;
    }
    public MasqMotorSystem runWithoutEncoders() {
        for (MasqMotor masqMotor: motors)
            masqMotor.runWithoutEncoders();
        return this;
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
    public double getAcceleration () {
        double i = 0;
        double rate = 0;
        for (MasqMotor masqMotor: motors){
            rate += masqMotor.getAcceleration();
            i++;
        }
        return rate/i;
    }
    public boolean isBusy() {
        boolean isBusy = false;
        for (MasqMotor masqMotor: motors)
            isBusy = masqMotor.isBusy();
        return isBusy;
    }
    public double getCurrentPosition() {
        int total = 0;
        for (MasqMotor m : motors) total += m.getCurrentPosition();
        return total / numMotors;
    }
    public double getAbsolutePosition ()  {
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