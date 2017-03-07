package BasicLib4997.MasqMotors.MasqRobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;

import BasicLib4997.PID_Constants;
import BasicLib4997.DashBoard;
import BasicLib4997.MasqHardware;
import BasicLib4997.MasqMotors.MasqMotor;
import BasicLib4997.MasqMotors.MasqTankDrive;
import BasicLib4997.MasqSensors.AdafruitIMU;
import BasicLib4997.MasqSensors.MasqClock;
import BasicLib4997.MasqSensors.MasqColorSensor;
import BasicLib4997.MasqSensors.MasqMatiboxUltraSensor;
import BasicLib4997.MasqSensors.MasqODS;
import BasicLib4997.MasqSensors.Sensor_Thresholds;
import BasicLib4997.MasqServos.MasqServo;
import static BasicLib4997.MasqMotors.MasqTankDrive.convert;

/**
 * Created by Archish on 10/28/16.
 */

public class MasqRobot implements PID_Constants, Sensor_Thresholds, MasqHardware {
    // MasqMotor and MasqMotor Systems
    public MasqTankDrive driveTrain = new MasqTankDrive("leftFront", "leftBack", "rightFront", "rightBack");
    public MasqMotor shooter1 = new MasqMotor("flicker1");
    public MasqMotor shooter2 = new MasqMotor("flicker2");
    ///MasqClock
    //Servos
    public MasqServo rightPresser = new MasqServo("servo_blue");
    public MasqServo leftPresser = new MasqServo("servo_red");
    public MasqMotor collector = new MasqMotor("collector");
    public MasqServo indexer = new MasqServo("ball_stop");
    //IMU
    public MasqMatiboxUltraSensor ultra = new MasqMatiboxUltraSensor("mata");
    public AdafruitIMU imu = new AdafruitIMU("imu");
    public MasqODS ods = new MasqODS("ods");
    public MasqColorSensor leftColor = new MasqColorSensor("sensor_color", 60);
    //RangeSensor
    private static final int DEFAULT_SLEEP_TIME = 500;
    private static final double DEFAULT_TIMEOUT = 3;
    public double angleLeftCover = 0;
    private boolean opModeIsActive() {
        return ((LinearOpMode) (FtcOpModeRegister.opModeManager.getActiveOpMode())).opModeIsActive();
    }
    public void drive(double power, int distance, Direction DIRECTION, int sleepTime, double targetAngle) {
        double angle = targetAngle;
        int newDistance = convert(distance);
        driveTrain.resetEncoders();
        driveTrain.setDistance((int)((-newDistance) * DIRECTION.value));
        driveTrain.runToPosition();
        while (driveTrain.rightIsBusy() && opModeIsActive()) {
            double newPowerLeft = power;
            double imuVal = imu.getHeading();
            double error = angle - imuVal;
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
    public void drive(double power, int distance, Direction DIRECTION) {
        drive(power, distance, DIRECTION, DEFAULT_SLEEP_TIME);
    }
    public void turn(int angle, Direction DIRECTION, double timeout, double ki) {
        double targetAngle = imu.adjustAngle(imu.getHeading() + (DIRECTION.value * angle));
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
            double errorkp = currentError * KP_TURN;
            double integralki = currentError * ki * tChange;
            double dervitive = (currentError - prevError) / tChange;
            double dervitivekd = dervitive * KD_TURN;
            newPower = (errorkp + integralki + dervitivekd);
            driveTrain.setPowerRight(-newPower);
            driveTrain.setPowerLeft(newPower);
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
            driveTrain.setPowerRight(-newPower);
            driveTrain.setPowerLeft(newPower);
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
    public void stopRed(double power, Direction Direction) {
        driveTrain.runUsingEncoder();
        double targetAngle = imu.getHeading();
        while (!(leftColor.isRed()) && opModeIsActive()) {
            double newPower = power;
            double heading = imu.getHeading();
            double error = targetAngle - heading;
            double errorkp = error * KP_STRAIGHT;
            newPower = newPower - (errorkp * Direction.value);
            driveTrain.setPowerLeft(power * Direction.value);
            driveTrain.setPowerRight(newPower * Direction.value);
            DashBoard.getDash().create("Heading", heading);
            DashBoard.getDash().create("red Val", leftColor.colorNumber());
            DashBoard.getDash().update();
        }
        driveTrain.StopDriving();
    }
    public void stopBlue(double power, Direction Direction) {
        driveTrain.runUsingEncoder();
        double targetAngle = imu.getHeading();
        while ((!leftColor.isBlue()) && opModeIsActive()){
            double newPower = power;
            double heading = imu.getHeading();
            double error = targetAngle - heading;
            double errorkp = error * KP_STRAIGHT;
            newPower = newPower - (errorkp * Direction.value);
            driveTrain.setPowerLeft(power * Direction.value);
            driveTrain.setPowerRight(newPower * Direction.value);
            DashBoard.getDash().create("Heading", heading);
            DashBoard.getDash().create("Blue Val", leftColor.colorNumber());
            DashBoard.getDash().update();
        }
        driveTrain.StopDriving();
    }
    public void stopRedRight(double power, Direction Direction) {
        driveTrain.runUsingEncoder();
        double targetAngle = imu.getHeading();
        while (!(leftColor.isRed()) && opModeIsActive()) {
            double newPower = power;
            double heading = imu.getHeading();
            double error = targetAngle - heading;
            double errorkp = error * KP_STRAIGHT;
            newPower = newPower - (errorkp * Direction.value);
            driveTrain.setPowerLeft(power * Direction.value);
            driveTrain.setPowerRight(newPower * Direction.value);
            DashBoard.getDash().create("Heading", heading);
            DashBoard.getDash().create("red Val", leftColor.colorNumber());
            DashBoard.getDash().update();
        }
        driveTrain.StopDriving();
    }
    public void stopBlueRight(double power, Direction Direction) {
        driveTrain.runUsingEncoder();
        double targetAngle = imu.getHeading();
        while ((!leftColor.isBlue()) && opModeIsActive()){
            double newPower = power;
            double heading = imu.getHeading();
            double error = targetAngle - heading;
            double errorkp = error * KP_STRAIGHT;
            newPower = newPower - (errorkp * Direction.value);
            driveTrain.setPowerLeft(power * Direction.value);
            driveTrain.setPowerRight(newPower * Direction.value);
            DashBoard.getDash().create("Heading", heading);
            DashBoard.getDash().create("Blue Val", leftColor.colorNumber());
            DashBoard.getDash().update();
        }
        driveTrain.StopDriving();
    }
    public void stopWhite (double power, Direction Direction) {
        driveTrain.runUsingEncoder();
        double targetAngle = imu.getHeading();
        while ((!ods.isWhite()) && opModeIsActive()) {
            double newPower = power;
            double heading = imu.getHeading();
            double error = targetAngle - heading;
            double errorkp = error * KP_STRAIGHT;
            newPower = newPower - (errorkp * Direction.value);
            driveTrain.setPowerLeft(power * Direction.value);
            driveTrain.setPowerRight(newPower * Direction.value);
            DashBoard.getDash().create("Heading", heading);
            DashBoard.getDash().create("red Val", leftColor.colorNumber());
            DashBoard.getDash().update();
        }
        driveTrain.StopDriving();
    }
    //rangeSensor
    //setPower
    String shooterDash = shooter1.getDash() + shooter2.getDash();
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

    public String getDash() {return imu.getDash() +  leftColor.getDash() +  shooterDash + collector.getDash() + driveTrain.getDash();
    }
}
