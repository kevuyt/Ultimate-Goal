package BasicLib4997.MasqMotors;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;

import com.qualcomm.robotcore.hardware.DcMotorController;

import BasicLib4997.DashBoard;
import BasicLib4997.MasqHardware;
import BasicLib4997.MasqMotors.MasqRobot.Constants;
import BasicLib4997.MasqMotors.MasqRobot.MasqRobot;
import BasicLib4997.MasqSensors.MasqClock;

/**
 * This is a custom motor that includes stall detection and telemetry
 */
public class MasqMotor implements Constants, MasqHardware{
    private DcMotor motor;
    private String nameMotor;
    private MasqClock clock = new MasqClock();
    private double tChange;
    private double prevPos= 0;
    private double previousTime = 0;
    private double startTime = System.nanoTime();
    public MasqMotor(String name){
        this.nameMotor = name;
        motor = FtcOpModeRegister.opModeManager.getHardwareMap().dcMotor.get(name);
    }
    public MasqMotor(String name, DcMotor.Direction direction) {
        this.nameMotor = name;
        motor = FtcOpModeRegister.opModeManager.getHardwareMap().dcMotor.get(name);
        motor.setDirection(direction);
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
    public void setbrakeMode () {
        setPower(0.001);
    }
    public double getCurrentPos () {
        double currentPos;
        currentPos = motor.getCurrentPosition();
        return currentPos;
    }
    public double getPower() {
        return motor.getPower();
    }
    public double getRate () {
        double posC = getCurrentPos() - prevPos;
        sleep(100);
        double tChange = System.nanoTime() - previousTime;
        previousTime = System.nanoTime();
        tChange = tChange / 1e9;
        prevPos = getCurrentPos();
        return posC / tChange;
    }
    public DcMotorController getController() {
        return motor.getController();
    }
    public void sleep (int sleepTime) {
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    public void telemetryRun (boolean showCurrentPos) {
        DashBoard.getDash().create(nameMotor + "telemetry");
        DashBoard.getDash().create("isStalled", isStalled());
        DashBoard.getDash().create("isBusy", isBusy());
        if (showCurrentPos) {
        DashBoard.getDash().create("Current Position", getCurrentPos());
        }
    }

    public String getName() {
        return nameMotor;
    }

    public String getDash() {
        return "Current Position" + Double.toString(getCurrentPos());
    }
}

