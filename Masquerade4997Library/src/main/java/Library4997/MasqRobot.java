package Library4997;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;

import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqMotors.MasqTankDrive;
import Library4997.MasqSensors.MasqLimitSwitch;
import Library4997.MasqSensor;
import Library4997.MasqServos.MasqCRServo;
import Library4997.MasqWrappers.Direction;
import Library4997.PID_Constants;
import Library4997.MasqWrappers.DashBoard;
import Library4997.MasqSensors.MasqAdafruitIMU;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqSensors.MasqColorSensor;
import Library4997.MasqSensors.MasqODS;
import Library4997.MasqServos.MasqServo;
import static Library4997.MasqMotors.MasqMotorSystem.convert;

/**
 * MasqRobot--> Contains all hardware and methods to run the robot.k
 */

public class MasqRobot implements PID_Constants {
    //////////////////////////////PlaceAllHardwareHere/////////////////////////////////////
    public MasqTankDrive driveTrain = new MasqTankDrive("leftFront", "leftBack", "rightFront", "rightBack");

    public MasqMotor collector = new MasqMotor("collector");
    public MasqMotor shooter = new MasqMotor("shooter", MasqMotor.Rate.RUN);
    public MasqMotor lights = new MasqMotor("light");
    public MasqServo indexer = new MasqServo("indexer");

    public MasqCRServo rightPresser = new MasqCRServo("rightPresser");
    public MasqCRServo leftPresser = new MasqCRServo("leftPresser");

    public MasqLimitSwitch limitSwitch = new MasqLimitSwitch("l");
    public MasqAdafruitIMU imu = new MasqAdafruitIMU("imu");
    public MasqODS ods = new MasqODS("ods");

    public MasqColorSensor rightColor = new MasqColorSensor("rightColor" , 62);
    public MasqColorSensor colorRejection = new MasqColorSensor("colorRejection", 64);
    public MasqColorSensor leftColor = new MasqColorSensor("leftColor", 60);

    private MasqClock timeoutClock = new MasqClock();
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    private static final int DEFAULT_SLEEP_TIME = 500;
    private static final double DEFAULT_TIMEOUT = 3;
    public double angleLeftCover = 0;
    public double color = 1;

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
    public void drive(int distance, double power, Direction DIRECTION, double targetAngle, int sleepTime, double timeOut) {
        int newDistance = convert(distance);
        driveTrain.resetEncoders();
        driveTrain.setDistance((int)((-newDistance) * DIRECTION.value));
        driveTrain.runToPosition();
        timeoutClock.reset();
        while (driveTrain.rightIsBusy() && opModeIsActive() && !timeoutClock.elapsedTime(timeOut, MasqClock.Resolution.SECONDS)) {
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
        driveTrain.stopDriving();
        driveTrain.runUsingEncoder();
        sleep(sleepTime);
    }
    public void drive(int distance, double power, Direction DIRECTION, double timeOut, int sleepTime) {
        double targetAngle = imu.getHeading();
        int newDistance = convert(distance);
        driveTrain.resetEncoders();
        driveTrain.setDistance((int)((-newDistance) * DIRECTION.value));
        driveTrain.runToPosition();
        timeoutClock.reset();
        while (driveTrain.rightIsBusy() && opModeIsActive() && !timeoutClock.elapsedTime(timeOut, MasqClock.Resolution.SECONDS)) {
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
        driveTrain.stopDriving();
        driveTrain.runUsingEncoder();
        sleep(sleepTime);
    }
    public void drive(int distance, double power, Direction DIRECTION, double timeOut) {
        drive(distance, power, DIRECTION, timeOut, DEFAULT_SLEEP_TIME);
    }
    public void drive(int distance, double power, Direction Direction) {
        drive(distance, power, Direction, DEFAULT_TIMEOUT);
    }
    public void drive (int distance, double power){
        drive(distance, power, Direction.FORWARD);
    }
    public void drive(int distance) {
        drive(distance, 0.5);
    }

    public void turn(int angle, Direction DIRECTION, double timeOut, int sleepTime, double kp, double ki, double kd) {
        double targetAngle = imu.adjustAngle(imu.getHeading() + (DIRECTION.value * angle));
        double acceptableError = 0.5;
        double currentError = 1;
        double prevError = 0;
        double integral = 0;
        double newPower = 0;
        double previousTime = 0;
        timeoutClock.reset();
        while (opModeIsActive() && (imu.adjustAngle(Math.abs(currentError)) > acceptableError) && !timeoutClock.elapsedTime(timeOut, MasqClock.Resolution.SECONDS)) {
            double tChange = System.nanoTime() - previousTime;
            previousTime = System.nanoTime();
            tChange = tChange / 1e9;
            double imuVAL = imu.getHeading();
            currentError = imu.adjustAngle(targetAngle - imuVAL);
            integral += currentError * ID;
            double errorkp = currentError * kp;
            double integralki = currentError * ki * tChange;
            double dervitive = (currentError - prevError) / tChange;
            double dervitivekd = dervitive * kd;
            newPower = (errorkp + integralki + dervitivekd);
            driveTrain.setPower(newPower, -newPower);
            prevError = currentError;
            this.angleLeftCover = currentError;
            DashBoard.getDash().create("TargetAngle", targetAngle);
            DashBoard.getDash().create("Heading", imuVAL);
            DashBoard.getDash().create("AngleLeftToCover", currentError);
            DashBoard.getDash().update();
        }
        driveTrain.stopDriving();
        sleep(sleepTime);
    }
    public void turn(int angle, Direction DIRECTION, double timeOut, int sleepTime, double kp, double ki) {
        turn(angle, DIRECTION, timeOut, sleepTime, kp, ki, KD_TURN);
    }
    public void turn(int angle, Direction DIRECTION, double timeOut, int sleepTime, double kp) {
        turn(angle, DIRECTION, timeOut, sleepTime, kp, KI_TURN);
    }
    public void turn(int angle, Direction DIRECTION, double timeOut, int sleepTime) {
        turn(angle, DIRECTION, timeOut, sleepTime, KP_TURN);
    }
    public void turn( int angle, Direction DIRECTION, double timeout)  {
        turn(angle, DIRECTION, timeout, DEFAULT_SLEEP_TIME);
    }
    public void turn(int angle, Direction DIRECTION)  {
        turn(angle, DIRECTION, DEFAULT_TIMEOUT);
    }

    public int getDelta (double measureOne, Direction direction) {
        return (int) (measureOne - (imu.getHeading() * direction.value));
    }
    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : FtcOpModeRegister.opModeManager.getHardwareMap().voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    public void stopRed(MasqColorSensor colorSensor, double power, Direction Direction) {
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
        driveTrain.stopDriving();
    }
    public void stopRed (MasqColorSensor colorSensor, double power){
        stopRed(colorSensor, power, Direction.BACKWARD);
    }
    public void stopRed (MasqColorSensor colorSensor){
        stopRed(colorSensor, 0.5);
    }

    public void stopBlue(MasqColorSensor colorSensor, double power, Direction Direction) {
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
        driveTrain.stopDriving();
    }
    public void stopBlue (MasqColorSensor colorSensor, double power){
        stopBlue(colorSensor, power, Direction.BACKWARD);
    }
    public void stopBlue (MasqColorSensor colorSensor){
        stopBlue(colorSensor, 0.5);
    }

    public void stop(MasqSensor sensor, double power, Direction Direction) {
        driveTrain.runUsingEncoder();
        double targetAngle = imu.getHeading();
        while (sensor.stop() && opModeIsActive()) {
            double newPower = power;
            double heading = imu.getHeading();
            double error = targetAngle - heading;
            double errorKP = error * KP_STRAIGHT;
            newPower = newPower - (errorKP * Direction.value);
            driveTrain.setPowerLeft(newPower * Direction.value);
            driveTrain.setPowerRight(power * Direction.value);
            DashBoard.getDash().create("is Stopped", sensor.stop());
            DashBoard.getDash().update();
        }
        driveTrain.stopDriving();
    }
    public void stop (MasqSensor sensor, double power){
        stop(sensor, power, Direction.BACKWARD);
    }
    public void stop (MasqSensor sensor){
        stop(sensor, 0.5);
    }

    public void sleep(int time) {
        try {
            Thread.sleep((long) time);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
    public void sleep() {
        sleep(DEFAULT_SLEEP_TIME);
    }

    public void move(boolean control, MasqMotor motor, double power){
        if (control){
            motor.setPower(power);
        }
        else if (!control){
            motor.setPower(0);
        }
    }
    public void move(boolean forwardControl, boolean backwordControl, MasqMotor motor, double power){
        if (forwardControl){
            motor.setPower(power);
        }
        else if (backwordControl){
            motor.setPower(-power);
        }
        else {
            motor.setPower(0);
        }
    }
    public void move(boolean control, MasqServo motor, double position, double zeroPosition){
        if (control){
            motor.setPosition(position);
        }
        else if (!control){
            motor.setPosition(zeroPosition);
        }
    }
    public void move(boolean control, boolean backwordControl, MasqServo motor, double position, double zeroPosition){
        if (control){
            motor.setPosition(position);
        }
        else if (backwordControl){
            motor.setPosition(zeroPosition);
        }
    }
    public void move(boolean forwardControl, boolean backwordControl, MasqCRServo servo, double power){
        if (forwardControl){
            servo.setPower(power);
        }
        else if (backwordControl){
            servo.setPower(-power);
        }
        else {
            servo.setPower(0);
        }
    }
    public void move(MasqMotor motor, double power){
        motor.setPower(power);
    }

}
