package Library4997.MasqMotors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

import Library4997.MasqResources.MasqHelpers.MasqEncoderModel;
import Library4997.MasqResources.MasqHelpers.MasqHardware;

/**
 * MasqMotorSystem That supports two or more motors and treats them as one
 */
public class MasqMotorSystem implements MasqHardware {
    public MasqMotor motor1 , motor2, motor3;
    private List<MasqMotor> motors;
    private int numMotors;
    private double currentPower = 0;
    private double slowDown = 0;
    private String systemName;
    public MasqMotorSystem(String name1, DcMotor.Direction direction, String name2, DcMotor.Direction direction2, String systemName, HardwareMap hardwareMap) {
        this.systemName = systemName;
        motor1 = new MasqMotor(name1, MasqEncoderModel.NEVEREST20, direction, hardwareMap);
        motor2 = new MasqMotor(name2, MasqEncoderModel.NEVEREST20, direction2, hardwareMap);
        motor3 = null;
        motors = Arrays.asList(motor1, motor2);
        numMotors = 2;
    }
    public MasqMotorSystem(String name1, String name2, String systemName, HardwareMap hardwareMap) {
        this.systemName = systemName;
        motor1 = new MasqMotor(name1, MasqEncoderModel.NEVEREST20, DcMotor.Direction.FORWARD, hardwareMap);
        motor2 = new MasqMotor(name2, MasqEncoderModel.NEVEREST20, DcMotor.Direction.FORWARD, hardwareMap);
        motor3 = null;
        motors = Arrays.asList(motor1, motor2);
        numMotors = 2;
    }
    public MasqMotorSystem(String name1, DcMotor.Direction direction,
                           String name2, DcMotor.Direction direction2,
                           String name3, DcMotor.Direction direction3, String systemName,
                            HardwareMap hardwareMap) {
        this.systemName = systemName;
        motor1 = new MasqMotor(name1, MasqEncoderModel.NEVEREST20, direction, hardwareMap);
        motor2 = new MasqMotor(name2, MasqEncoderModel.NEVEREST20, direction2, hardwareMap);
        motor3 = new MasqMotor(name3, MasqEncoderModel.NEVEREST20, direction3, hardwareMap);
        motors = Arrays.asList(motor1, motor2, motor3);
        numMotors = 3;
    }
    public MasqMotorSystem(String name1, String name2, String name3, String systemName, HardwareMap hardwareMap) {
        this.systemName = systemName;
        motor1 = new MasqMotor(name1, MasqEncoderModel.NEVEREST20, DcMotor.Direction.FORWARD, hardwareMap);
        motor2 = new MasqMotor(name2, MasqEncoderModel.NEVEREST20, DcMotor.Direction.FORWARD, hardwareMap);
        motor3 = new MasqMotor(name3, MasqEncoderModel.NEVEREST20, DcMotor.Direction.FORWARD, hardwareMap);
        motors = Arrays.asList(motor1, motor2, motor3);
        numMotors = 3;
    }
    public MasqMotorSystem resetEncoders() {
        for (MasqMotor masqMotor : motors)
            masqMotor.resetEncoder();
        return this;
    }
    public MasqMotorSystem setKp(double kp){
        for (MasqMotor masqMotor: motors) masqMotor.setKp(kp);
        return this;
    }
    public MasqMotorSystem setKi(double ki){
        for (MasqMotor masqMotor: motors) masqMotor.setKi(ki);
        return this;
    }
    public double getPower() {
        double num = 0, sum = 0;
        for (MasqMotor masqMotor: motors) {
            sum += masqMotor.getPower();
            num++;
        }
        return sum/num;
    }
    public MasqMotorSystem setKd(double kd){
        for (MasqMotor masqMotor: motors) masqMotor.setKd(kd);
        return this;
    }
    public void setVelocity(double power) {
        currentPower = power;
        for (MasqMotor masqMotor : motors)masqMotor.setVelocity(power);
    }
    public void setPower(double power) {
        for (MasqMotor masqMotor : motors)masqMotor.setPower(power);
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
    public MasqMotorSystem runWithoutEncoders() {
        for (MasqMotor masqMotor: motors)
            masqMotor.runWithoutEncoders();
        return this;
    }
    public MasqMotorSystem setClosedLoop (boolean closedLoop){
        for (MasqMotor masqMotor: motors)
            masqMotor.setClosedLoop(closedLoop);
        return this;
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
    public void stopDriving() {
        setVelocity(0);
    }
    public static int convert(int TICKS) {
        return (int) ((TICKS * 35.1070765836));
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

    public String getName() {
        return systemName;
    }
    public String[] getDash() {
        return new String[]{ "Current Position" + getCurrentPosition()};
    }
}