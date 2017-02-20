package BasicLib4997.Motors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import org.firstinspires.ftc.robotcore.internal.TelemetryImpl;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import BasicLib4997.Motors.TankDrive.PID_Constants;
import BasicLib4997.Motors.TankDrive.TankDrive;
import BasicLib4997.Sensors.Clock;

/**
 * This is a custom motor that includes stall detection and telemetry
 */
public class Motor implements PID_Constants{
    private DcMotor motor;
    private String nameMotor;
    private Clock clock = new Clock();
    private double tChange;
    private double prevPos= 0;
    private double previousTime = 0;
    private double startTime = System.nanoTime();
    public Motor(String name){
        this.nameMotor = name;
        motor = FtcOpModeRegister.opModeManager.getHardwareMap().dcMotor.get(name);
    }
    public Motor(String name, DcMotor.Direction direction) {
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
    double getCurrentPos () {
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
        TankDrive.getTelemetry().addTelemetry(nameMotor + "telemetry");
        TankDrive.getTelemetry().addTelemetry("isStalled", isStalled());
        TankDrive.getTelemetry().addTelemetry("isBusy", isBusy());
        if (showCurrentPos) {
        TankDrive.getTelemetry().addTelemetry("Current Position", getCurrentPos());
        }
    }
}

