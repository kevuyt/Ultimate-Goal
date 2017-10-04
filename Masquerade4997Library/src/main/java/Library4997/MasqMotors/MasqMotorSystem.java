package Library4997.MasqMotors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

import Library4997.MasqHardware;
import Library4997.MasqExternal.PID_CONSTANTS;

/**
 * MasqMotorSystem That supports two or more motors and treats them as one
 */
public class MasqMotorSystem implements PID_CONSTANTS, MasqHardware {
    public MasqMotor motor1 , motor2, motor3;
    private List<MasqMotor> motors;
    private int numMotors;
    private String systemName;
    public MasqMotorSystem(String name1, DcMotor.Direction direction, String name2, DcMotor.Direction direction2, String systemName, HardwareMap hardwareMap) {
        this.systemName = systemName;
        motor1 = new MasqMotor(name1, direction, hardwareMap);
        motor2 = new MasqMotor(name2, direction2, hardwareMap);
        motor3 = null;
        motors = Arrays.asList(motor1, motor2);
        numMotors = 2;
    }
    public MasqMotorSystem(String name1, String name2, String systemName, HardwareMap hardwareMap) {
        this.systemName = systemName;
        motor1 = new MasqMotor(name1, DcMotor.Direction.FORWARD, hardwareMap);
        motor2 = new MasqMotor(name2, DcMotor.Direction.FORWARD, hardwareMap);
        motor3 = null;
        motors = Arrays.asList(motor1, motor2);
        numMotors = 2;
    }
    public MasqMotorSystem(String name1, DcMotor.Direction direction,
                           String name2, DcMotor.Direction direction2,
                           String name3, DcMotor.Direction direction3, String systemName,
                            HardwareMap hardwareMap) {
        this.systemName = systemName;
        motor1 = new MasqMotor(name1, direction, hardwareMap);
        motor2 = new MasqMotor(name2, direction2, hardwareMap);
        motor3 = new MasqMotor(name3, direction3, hardwareMap);
        motors = Arrays.asList(motor1, motor2, motor3);
        numMotors = 3;
    }
    public MasqMotorSystem(String name1, String name2, String name3, String systemName, HardwareMap hardwareMap) {
        this.systemName = systemName;
        motor1 = new MasqMotor(name1, DcMotor.Direction.FORWARD, hardwareMap);
        motor2 = new MasqMotor(name2, DcMotor.Direction.FORWARD, hardwareMap);
        motor3 = new MasqMotor(name3, DcMotor.Direction.FORWARD, hardwareMap);
        motors = Arrays.asList(motor1, motor2, motor3);
        numMotors = 3;
    }
    public MasqMotorSystem resetEncoder () {
        for (MasqMotor masqMotor : motors)
            masqMotor.resetEncoder();
        return this;
    }
    public void setPower (double power) {
        for (MasqMotor masqMotor : motors)
            masqMotor.setPower(power);
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
    public double getRate(){
        double i = 1;
        double rate = 0;
        for (MasqMotor masqMotor: motors){
            rate += masqMotor.getRate() / i;
            i++;
        }
        return rate;
    }
    public void stopDriving() {
        setPower(0);
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

    public String getName() {
        return systemName;
    }
    public String[] getDash() {
        return new String[]{ "Current Position" + getCurrentPosition()};
    }
}