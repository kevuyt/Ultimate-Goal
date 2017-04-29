package Library4997.MasqMotors;

import android.widget.TabHost;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;

import com.qualcomm.robotcore.hardware.DcMotorController;

import javax.xml.transform.sax.TemplatesHandler;

import Library4997.MasqHardware;
import Library4997.PID_Constants;
import Library4997.MasqSensors.MasqClock;

/**
 * This is a custom motor that includes stall detection and telemetry
 */
public class MasqMotor implements PID_Constants, MasqHardware{
    private DcMotor motor;
    private String nameMotor;
    private MasqClock clock = new MasqClock();
    private double tChange;
    private double prevPos= 0;
    private double previousTime = 0;
    private double startTime = System.nanoTime();
    private double rate = 0;
    private RateThread rateThread = new RateThread();
    public MasqMotor(String name){
        this.nameMotor = name;
        motor = FtcOpModeRegister.opModeManager.getHardwareMap().dcMotor.get(name);

    }
    public MasqMotor(String name, DcMotor.Direction direction) {
        this.nameMotor = name;
        motor = FtcOpModeRegister.opModeManager.getHardwareMap().dcMotor.get(name);
        motor.setDirection(direction);
    }
    public MasqMotor(String name, Rate rate){
        this.nameMotor = name;
        motor = FtcOpModeRegister.opModeManager.getHardwareMap().dcMotor.get(name);
        if (rate.value)
        rateThread.run();
    }
    public MasqMotor(String name, DcMotor.Direction direction, Rate rate) {
        this.nameMotor = name;
        motor = FtcOpModeRegister.opModeManager.getHardwareMap().dcMotor.get(name);
        motor.setDirection(direction);
        if (rate.value)
        rateThread.run();
    }
    public enum Rate {
        RUN (true),
        MINIMIZE(false);
        public final boolean value;
        Rate (boolean value) {this.value = value;}
    }
    public void runWithoutEncoders () {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    boolean isStalled () {
        double prevPosition = motor.getCurrentPosition();
        boolean isStalled;
        if (motor.getCurrentPosition() > prevPosition) {
            isStalled = false;
        }
        else if(motor.getCurrentPosition() - 10 > prevPosition) {
            isStalled = false;
        }
        else {
            isStalled = true;
        }
        return isStalled;

    }
    public double getRate () {
        return rate;
    }
    public void resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setPower (double power) {
        motor.setPower(power);
    }
    public void setDistance(int distance){
        motor.setTargetPosition(distance);
    }
    public void runUsingEncoder() {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void runToPosition(){
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    boolean isBusy () {
        return motor.isBusy();
    }
    public void setBrakeMode() {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public double getCurrentPos () {
         return motor.getCurrentPosition();
    }
    public double getPower() {
        return motor.getPower();
    }
    public void sleep (int sleepTime) {
        try {
            Thread.sleep(sleepTime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    public String getName() {
        return nameMotor;
    }
    public String[] getDash() {
        return new String[]{
                "Current Position" + Double.toString(getCurrentPos())
        };
    }
    private class RateThread implements Runnable{
        @Override
        public void run() {
            double positionChange = getCurrentPos() - prevPos;
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            double timeChange = System.nanoTime() - previousTime;
            previousTime = System.nanoTime();
            timeChange = timeChange / 1e9;
            prevPos = getCurrentPos();
            rate = positionChange / timeChange;
        }
    }
}

