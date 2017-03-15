package BasicLib4997.MasqRobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;

import java.util.Arrays;

import BasicLib4997.MasqMotors.MasqMotor;
import BasicLib4997.MasqMotors.MasqTankDrive;
import BasicLib4997.MasqSensors.MasqLimitSwitch;
import BasicLib4997.MasqSensors.MasqRangeSensor;
import BasicLib4997.MasqSensors.MasqTouchSensor;
import BasicLib4997.MasqServos.MasqCRServo;
import BasicLib4997.PID_Constants;
import BasicLib4997.DashBoard;
import BasicLib4997.MasqHardware;
import BasicLib4997.MasqMotors.MasqMotorSystem;
import BasicLib4997.MasqSensors.MasqAdafruitIMU;
import BasicLib4997.MasqSensors.MasqClock;
import BasicLib4997.MasqSensors.MasqColorSensor;
import BasicLib4997.MasqSensors.MasqODS;
import BasicLib4997.MasqSensors.Sensor_Thresholds;
import BasicLib4997.MasqServos.MasqServo;
import static BasicLib4997.MasqMotors.MasqMotorSystem.convert;

/**
 * The MasqRobot Because Masquerade is the Best
 */

public class MasqRobot implements PID_Constants, Sensor_Thresholds, MasqHardware {
    public MasqTankDrive driveTrain = new MasqTankDrive("leftFront", "leftBack", "rightFront", "rightBack");
    //private MasqMotor collector = new MasqMotor("collector");
    public MasqMotorSystem shooter = new MasqMotorSystem("shooter", "shooter2", "shooter");
    ///MasqClock
    //Servos
    public MasqLimitSwitch limitSwitch = new MasqLimitSwitch("l");
    private MasqCRServo rightPresser = new MasqCRServo("rightPresser");
    private MasqCRServo leftPresser = new MasqCRServo("leftPresser");
    private MasqMotor collector = new MasqMotor("collector");
    private MasqServo indexer = new MasqServo("indexer");
    //IMU
    public MasqAdafruitIMU imu = new MasqAdafruitIMU("imu");
    public MasqODS ods = new MasqODS("ods");
    //ColorSensor
    public MasqColorSensor rightColor = new MasqColorSensor("rightColor" , 62);
    public MasqColorSensor colorRejection = new MasqColorSensor("colorRejection", 64);
    public MasqColorSensor leftColor = new MasqColorSensor("leftColor", 60);
    //RangeSensor
    public MasqRangeSensor rangeSensor = new MasqRangeSensor("rangeSensor");
    private static final int DEFAULT_SLEEP_TIME = 500;
    private static final double DEFAULT_TIMEOUT = 3;
    public double angleLeftCover = 0;
    private double color = 1;
    public enum AllianceColor {
        BLUE (-1.0),
        RED (+1.0);
        public final double color;
        AllianceColor (double color) {this.color = color;}
    }
    public void setAllianceColor(AllianceColor allianceColor){
        this.color = allianceColor.color;
    }
    private boolean opModeIsActive() {
        return ((LinearOpMode) (FtcOpModeRegister.opModeManager.getActiveOpMode())).opModeIsActive();
    }
    public void drive(double power, int distance, Direction DIRECTION, int sleepTime, double targetAngle) {
        int newDistance = convert(distance);
        driveTrain.resetEncoders();
        driveTrain.setDistance((int)((-newDistance) * DIRECTION.value));
        driveTrain.runToPosition();
        while (driveTrain.rightIsBusy() && opModeIsActive()) {
            double newPowerLeft = power;
            double imuVal = imu.getHeading();
            double error = targetAngle - imuVal;
            double errorkp = error * KP_STRAIGHT;
            newPowerLeft = (newPowerLeft - (errorkp) * DIRECTION.value);
            driveTrain.setPowerRight(power);
            driveTrain.setPowerLeft(newPowerLeft);
            DashBoard.getDash().create("Heading", imuVal);
            DashBoard.getDash().create("DistanceLeft", newDistance + driveTrain.getCurrentPos());
            DashBoard.getDash().update();
        }
        driveTrain.StopDriving();
        driveTrain.runUsingEncoder();
        sleep(sleepTime);
    }
    public void drive(double power, int distance, double targetAngle, Direction DIRECTION) {
        drive(power, distance, DIRECTION, DEFAULT_SLEEP_TIME, targetAngle);
    }
    public void drive(double power, int distance, Direction DIRECTION, int sleepTime) {
        double targetAngle = imu.getHeading();
        int newDistance = convert(distance);
        driveTrain.resetEncoders();
        driveTrain.setDistance((int)((-newDistance) * DIRECTION.value));
        driveTrain.runToPosition();
        while (driveTrain.rightIsBusy() && opModeIsActive()) {
            double newPowerLeft = power;
            double imuVal = imu.getHeading();
            double error = targetAngle - imuVal;
            double errorkp = error * KP_STRAIGHT;
            newPowerLeft = (newPowerLeft - (errorkp) * DIRECTION.value);
            driveTrain.setPowerLeft(newPowerLeft);
            driveTrain.setPowerRight(power);
            DashBoard.getDash().create("Heading", imuVal);
            DashBoard.getDash().create("DistanceLeft", newDistance + driveTrain.getCurrentPos());
            DashBoard.getDash().update();
        }
        driveTrain.StopDriving();
        driveTrain.runUsingEncoder();
        sleep(sleepTime);
    }
    public void drive(double power, int distance, Direction DIRECTION) {
        drive(power, distance, DIRECTION, DEFAULT_SLEEP_TIME);
    }
    public void turn(int angle, Direction DIRECTION, double timeout, double kp, double ki, double kd) {
        double targetAngle = imu.adjustAngle(imu.getHeading() + (DIRECTION.value * angle));
        targetAngle *= color;
        double acceptableError = 0.5;
        double currentError = 1;
        double prevError = 0;
        double integral = 0;
        double newPower = 0;
        double previousTime = 0;
        MasqClock clock = new MasqClock("clock");
        while (opModeIsActive() && (imu.adjustAngle(Math.abs(currentError)) > acceptableError) && !clock.elapsedTime(timeout, MasqClock.Resolution.SECONDS)) {
            double tChange = System.nanoTime() - previousTime;
            previousTime = System.nanoTime();
            tChange = tChange / 1e9;
            double imuVAL = imu.getHeading();
            currentError = imu.adjustAngle(targetAngle - imuVAL);
            integral += currentError  * ID;
            double errorkp = currentError * kp;
            double integralki = currentError * ki * tChange;
            double dervitive = (currentError - prevError) / tChange;
            double dervitivekd = dervitive * kd;
            newPower = (errorkp + integralki + dervitivekd);
            driveTrain.setPower(newPower, -newPower);
            prevError = currentError;
            DashBoard.getDash().create("TargetAngle", targetAngle);
            DashBoard.getDash().create("Heading", imuVAL);
            DashBoard.getDash().create("AngleLeftToCover", currentError);
            DashBoard.getDash().update();
        }
        driveTrain.StopDriving();
        sleep(1000);
    }
    public void turn(int angle, Direction DIRECTION, double timeOut, int sleepTime) {
        double targetAngle = imu.adjustAngle(imu.getHeading() + (DIRECTION.value * angle));
        targetAngle *= color;
        double acceptableError = 0.5;
        double currentError = 1;
        double prevError = 0;
        double integral = 0;
        double newPower = 0;
        double previousTime = 0;
        MasqClock clock = new MasqClock("clock");
        while (opModeIsActive() && (imu.adjustAngle(Math.abs(currentError)) > acceptableError) && !clock.elapsedTime(timeOut, MasqClock.Resolution.SECONDS)) {
            double tChange = System.nanoTime() - previousTime;
            previousTime = System.nanoTime();
            tChange = tChange / 1e9;
            double imuVAL = imu.getHeading();
            currentError = imu.adjustAngle(targetAngle - imuVAL);
            integral += currentError  * ID;
            double errorkp = currentError * KP_TURN;
            double integralki = currentError * KI_TURN * tChange;
            double dervitive = (currentError - prevError) / tChange;
            double dervitivekd = dervitive * KD_TURN;
            newPower = (errorkp + integralki + dervitivekd);
            driveTrain.setPower(newPower, -newPower);
            prevError = currentError;
            DashBoard.getDash().create("TargetAngle", targetAngle);
            DashBoard.getDash().create("Heading", imuVAL);
            DashBoard.getDash().create("AngleLeftToCover", currentError);
            angleLeftCover = currentError;
            DashBoard.getDash().update();
        }
        driveTrain.StopDriving();
        sleep(sleepTime);
    }
    public void turn( int angle, Direction DIRECTION, double timeout)  {
        turn(angle, DIRECTION, timeout, DEFAULT_SLEEP_TIME);
    }
    public void turn(int angle, Direction DIRECTION)  {
        turn(angle, DIRECTION, DEFAULT_TIMEOUT);
    }
    public void setBrakeMode(int time) {
        driveTrain.setBrakeMode();
        sleep(time);
    }
    public void stopRed(double power, Direction Direction, MasqColorSensor colorSensor) {
        driveTrain.runUsingEncoder();
        double targetAngle = imu.getHeading();
        while (!(colorSensor.isRed()) && opModeIsActive()) {
            double newPower = power;
            double heading = imu.getHeading();
            double error = targetAngle - heading;
            double errorkp = error * KP_STRAIGHT;
            newPower = newPower - (errorkp * Direction.value);
            driveTrain.setPowerLeft(newPower * Direction.value);
            driveTrain.setPowerRight(power * Direction.value);
            DashBoard.getDash().create("Heading", heading);
            DashBoard.getDash().create("red Val", colorSensor.colorNumber());
            DashBoard.getDash().update();
        }
        driveTrain.StopDriving();
    }
    public void stopBlue(double power, Direction Direction, MasqColorSensor colorSensor) {
        driveTrain.runUsingEncoder();
        double targetAngle = imu.getHeading();
        while ((!colorSensor.isBlue()) && opModeIsActive()){
            double newPower = power;
            double heading = imu.getHeading();
            double error = targetAngle - heading;
            double errorkp = error * KP_STRAIGHT;
            newPower = newPower - (errorkp * Direction.value);
            driveTrain.setPowerLeft(newPower * Direction.value);
            driveTrain.setPowerRight(power * Direction.value);
            DashBoard.getDash().create("Heading", heading);
            DashBoard.getDash().create("Blue Val", colorSensor.colorNumber());
            DashBoard.getDash().update();
        }
        driveTrain.StopDriving();
    }
    public void stopWhite (double power, Direction Direction, MasqODS ods) {
        driveTrain.runUsingEncoder();
        double targetAngle = imu.getHeading();
        while ((!ods.isWhite()) && opModeIsActive()) {
            double newPower = power;
            double heading = imu.getHeading();
            double error = targetAngle - heading;
            double errorkp = error * KP_STRAIGHT;
            newPower = newPower - (errorkp * Direction.value);
            driveTrain.setPowerLeft(newPower * Direction.value);
            driveTrain.setPowerRight(power * Direction.value);
            DashBoard.getDash().create("Heading", heading);
            DashBoard.getDash().create("IS White", ods.isWhite());
            DashBoard.getDash().update();
        }
        driveTrain.StopDriving();
    }
    public void stopTouch(double power, Direction Direction, MasqTouchSensor sensor) {
        driveTrain.runUsingEncoder();
        double targetAngle = imu.getHeading();
        while (!sensor.isPressed() && opModeIsActive()) {
            double newPower = power;
            double heading = imu.getHeading();
            double error = targetAngle - heading;
            double errorKP = error * KP_STRAIGHT;
            newPower = newPower - (errorKP * Direction.value);
            driveTrain.setPowerLeft(newPower * Direction.value);
            driveTrain.setPowerRight(power * Direction.value);
            DashBoard.getDash().create("is Pressed", sensor.isPressed());
            DashBoard.getDash().update();
        }
        driveTrain.StopDriving();
    }
    public void stopLimit(double power, Direction Direction, MasqLimitSwitch lswitch) {
        driveTrain.runUsingEncoder();
        double targetAngle = imu.getHeading();
        while (!lswitch.isPressed() && opModeIsActive()) {
            double newPower = power;
            double heading = imu.getHeading();
            double error = targetAngle - heading;
            double errorKP = error * KP_STRAIGHT;
            newPower = newPower - (errorKP * Direction.value);
            driveTrain.setPowerLeft(newPower * Direction.value);
            driveTrain.setPowerRight(power * Direction.value);
            DashBoard.getDash().create("is Presser", lswitch.isPressed());
            DashBoard.getDash().update();
        }
        driveTrain.StopDriving();
    }
    public void sleep() {
        sleep(1000);
    }
    public void sleep(int time) {
        try {
            Thread.sleep((long) time);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
    public String getName() {
        return "Robot";
    }
    public String[] getDash() {return new String[]{
            Arrays.toString(imu.getDash()),
            Arrays.toString(leftColor.getDash()),
    };}
}
