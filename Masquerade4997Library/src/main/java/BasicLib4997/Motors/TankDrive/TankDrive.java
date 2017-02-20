package BasicLib4997.Motors.TankDrive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import BasicLib4997.Motors.Motor;
import BasicLib4997.Motors.MotorSystem;
import BasicLib4997.Sensors.AdafruitIMU;
import BasicLib4997.Sensors.Clock;
import BasicLib4997.Sensors.I2CColorSensor;
import BasicLib4997.Sensors.MR_RangeSensor;
import BasicLib4997.Sensors.ODS;
import BasicLib4997.Sensors.Sensor_Thresholds;
import BasicLib4997.Servos.CR_Servo;
import BasicLib4997.Servos.Servo;
import static BasicLib4997.Motors.MotorSystem.convert;

/**
 * Created by Archish on 10/28/16.
 */

public class TankDrive implements PID_Constants, Sensor_Thresholds {

    public TankDrive(Telemetry telemetry){
        this.telemetry  = telemetry;
        instance = this;
    }
    public static TankDrive getTelemetry(){
        return instance;
    }
    private static TankDrive instance;
    private Telemetry telemetry;
    public void addTelemetry(String string) {
        telemetry.addLine(string);
    }
    public void addTelemetry(String string, Object data) {
        telemetry.addData(string, data);
    }
    public void addSticky(String string){
        telemetry.log().add(string);
        telemetry.update();
    }
    public void addSticky(String string, Object data){
        telemetry.log().add(string, data);
        telemetry.update();
    }
    // Motor and Motor Systems
    public MotorSystem driveTrain = new MotorSystem("leftFront", "leftBack", "rightFront", "rightBack");
    //private Motor collector = new Motor("collector");
    public Motor shooter = new Motor("shooter");
    public Motor shooter2 = new Motor("shooter2");
    ///Clock
    //Servos
    private CR_Servo rightPresser = new CR_Servo("rightPresser");
    private CR_Servo leftPresser = new CR_Servo("leftPresser");
    private Motor collector = new Motor("collector");
    private Servo indexer = new Servo("indexer");
    //IMU
    public AdafruitIMU imu = new AdafruitIMU("imu");
    public ODS ods = new ODS ("ods");
    //ColorSensor
    public I2CColorSensor rightColor = new I2CColorSensor("rightColor" , 62);
    public I2CColorSensor colorRejection = new I2CColorSensor("colorRejection", 64);
    public I2CColorSensor leftColor = new I2CColorSensor("leftColor", 60);
    //RangeSensor
    public MR_RangeSensor rangeSensor = new MR_RangeSensor("rangeSensor");
    private static final int DEFAULT_SLEEP_TIME = 1000;
    private static final double DEFAULT_TIMEOUT = 3;
    private boolean opModeIsActive() {
        return ((LinearOpMode) (FtcOpModeRegister.opModeManager.getActiveOpMode())).opModeIsActive();
    }
    public void drivePID(double power, int distance, Direction DIRECTION, int sleepTime, double targetAngle) {
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
            TankDrive.getTelemetry().addTelemetry("Heading", imuVal);
            TankDrive.getTelemetry().addTelemetry("DistanceLeft", newDistance + driveTrain.getCurrentPos());
            telemetry.update();
        }
        driveTrain.StopDriving();
        driveTrain.runUsingEncoder();
        sleep(sleepTime);
    }
    public void drivePID(double power, int distance, double targetAngle, Direction DIRECTION) {
        drivePID(power, distance, DIRECTION, DEFAULT_SLEEP_TIME, targetAngle);
    }
    public void drivePID(double power, int distance, Direction DIRECTION, int sleepTime) {
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
            TankDrive.getTelemetry().addTelemetry("Heading", imuVal);
            TankDrive.getTelemetry().addTelemetry("DistanceLeft", newDistance + driveTrain.getCurrentPos());
            telemetry.update();
        }
        driveTrain.StopDriving();
        driveTrain.runUsingEncoder();
        sleep(sleepTime);
    }
    public void drivePID(double power, int distance, Direction DIRECTION) {
        drivePID(power, distance, DIRECTION, DEFAULT_SLEEP_TIME);
    }
    public void turnPID(double power, int angle, Direction DIRECTION, double timeOut,  int sleepTime) {
        double targetAngle = imu.adjustAngle(imu.getHeading() + (DIRECTION.value * angle));
        double acceptableError = 0.5;
        double currentError = 1;
        double prevError = 0;
        double integral = 0;
        double newPower = power;
        double previousTime = 0;
        Clock clock = new Clock("clock");
        while (opModeIsActive() && (imu.adjustAngle(Math.abs(currentError)) > acceptableError) && !clock.elapsedTime(timeOut, Clock.Resolution.SECONDS)) {
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
            TankDrive.getTelemetry().addTelemetry("TargetAngle", targetAngle);
            TankDrive.getTelemetry().addTelemetry("Heading", imuVAL);
            TankDrive.getTelemetry().addTelemetry("AngleLeftToCover", currentError);
            telemetry.update();
        }
        driveTrain.StopDriving();
        sleep(sleepTime);
    }
    public void turnPID(double power, int angle, Direction DIRECTION, double timeout)  {
        turnPID(power, angle, DIRECTION, timeout, DEFAULT_SLEEP_TIME);
    }
    public void turnPID(double power, int angle, Direction DIRECTION)  {
        turnPID(power, angle, DIRECTION, DEFAULT_TIMEOUT);
    }
    public void drivePIDRange(double power, int distance, int sleepTime) {
        double targetAngle = imu.getHeading();
        driveTrain.runUsingEncoder();
        while (rangeSensor.rawUltrasonic() > distance && opModeIsActive()) {
            double newPowerLeft = power;
            double imuVal = imu.getHeading();
            double error = targetAngle - imuVal;
            double errorkp = error * KP_STRAIGHT;
            newPowerLeft = (newPowerLeft - (errorkp));
            driveTrain.setPowerRight(-power);
            driveTrain.setPowerLeft(-newPowerLeft);
            TankDrive.getTelemetry().addTelemetry("Heading", imuVal);
            TankDrive.getTelemetry().addTelemetry("Ultrasonic", rangeSensor.rawUltrasonic());
            telemetry.update();
        }
        driveTrain.StopDriving();
        driveTrain.runWithoutEncoders();
        sleep(sleepTime);
    }
    public void drivePIDRange(double power, int distance ) {
        drivePIDRange(power, distance, DEFAULT_SLEEP_TIME);
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
            TankDrive.getTelemetry().addTelemetry("Heading", heading);
            TankDrive.getTelemetry().addTelemetry("red Val", leftColor.colorNumber());
            telemetry.update();
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
            TankDrive.getTelemetry().addTelemetry("Heading", heading);
            TankDrive.getTelemetry().addTelemetry("Blue Val", leftColor.colorNumber());
            telemetry.update();
        }
        driveTrain.StopDriving();
    }
    public void stopWhite (double power, Direction Direction) {
        driveTrain.runUsingEncoder();
        double targetAngle = imu.getHeading();
        while ((!colorRejection.isWhite() || !ods.isWhite()) && opModeIsActive()) {
            double newPower = power;
            double heading = imu.getHeading();
            double error = targetAngle - heading;
            double errorkp = error * KP_STRAIGHT;
            newPower = newPower - (errorkp * Direction.value);
            driveTrain.setPowerLeft(power * Direction.value);
            driveTrain.setPowerRight(newPower * Direction.value);
            TankDrive.getTelemetry().addTelemetry("Heading", heading);
            TankDrive.getTelemetry().addTelemetry("red Val", leftColor.colorNumber());
            telemetry.update();
        }
        driveTrain.StopDriving();
    }
    //rangeSensor
    //setPower
    public void setPowerLeft(double power) {
        driveTrain.setPowerLeft(power);
    }
    public void setPowerRight(double power) {
        driveTrain.setPowerRight(power);
    }
    public void setPowerCollector(double powerCollector) {
        collector.setPower(powerCollector);
    }
    public void setPowerShooter(double powerShooter){
      shooter2.runWithoutEncoders();
      shooter.runWithoutEncoders();
      shooter2.setPower(powerShooter);
      shooter.setPower(powerShooter);
    }
    public void resetShooterEncoder () {
        shooter.resetEncoder();
        shooter2.resetEncoder();
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

    //servos
    public void setIndexer(double position) {
     indexer.setPosition(position);
    }
    public void setRightPresser(double power) {
        rightPresser.setPower(power);
    }
    public void setLeftPresser(double power) {
        leftPresser.setPower(power);
    }
    public void runAllTelemetry() {
        runSensorTelemetry();
        driveTrain.telemetryRun();
        collector.telemetryRun(false);
        shooter.telemetryRun(true);
        rightPresser.telemetryRun();
        indexer.telemetryRun();
    }
    public void runSensorTelemetry () {
        imu.telemetryRun();
        leftColor.telemetryRun();
        rightColor.telemetryRun();
        colorRejection.telemetryRun();
    }




}
