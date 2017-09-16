package Library4997;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqMotors.MasqTankDrive;
import Library4997.MasqSensors.MasqAdafruitIMU;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqSensors.MasqColorSensor;
import Library4997.MasqSensors.MasqVoltageSensor;
import Library4997.MasqSensors.MasqVuforia;
import Library4997.MasqServos.MasqCRServo;
import Library4997.MasqServos.MasqServo;
import Library4997.MasqWrappers.DashBoard;
import Library4997.MasqWrappers.Direction;
import Library4997.MasqWrappers.MasqController;

/**
 * MasqRobot--> Contains all hardware and methods to run the robot.
 */
//TODO make MasqRobot abstract to support multiple copies of a robot, for test bot, main bot, so forth
public class MasqRobot implements PID_CONSTANTS {
    ////////////////////////////// Place All Hardware Here ///////////////////////////////////////////////////
    public MasqTankDrive driveTrain = new MasqTankDrive("leftFront", "leftBack", "rightFront", "rightBack");

    public MasqAdafruitIMU imu = new MasqAdafruitIMU("imu");
    private MasqClock timeoutClock = new MasqClock();
    private MasqVoltageSensor voltageSensor = new MasqVoltageSensor();
    public MasqVuforia vuforia = new MasqVuforia("vumark-us1-t1", "vumark-us2-t1","vumark-us3-t3", "RelicRecoveryAssets");
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
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
    public enum Targets {
        TARGET_ONE("vumark-us1-t1"),
        TARGET_TWO ("vumark-us2-t1"),
        TARGET_THREE ("vumark-us3-t3");
        public final String value;
        Targets (String value) {this.value = value;}
    }
    public void setAllianceColor(AllianceColor allianceColor){
        this.color = allianceColor.color;
    }
    private boolean opModeIsActive() {
        return ((LinearOpMode) (FtcOpModeRegister.opModeManager.getActiveOpMode())).opModeIsActive();
    }
    public void drive(int distance, double speed, Direction DIRECTION, double timeOut, int sleepTime) {
        MasqClock timeoutTimer = new MasqClock();
        MasqClock loopTimer = new MasqClock();
        driveTrain.resetEncoders();
        double targetAngle = imu.getHeading();
        int targetClicks = (int)(distance * CLICKS_PER_CM);
        int clicksRemaining;
        double inchesRemaining;
        double angularError = imu.adjustAngle(targetAngle - imu.getHeading());
        double prevAngularError = angularError;
        double angularIntegral = 0;
        double angularDerivative;
        double powerAdjustment;
        double power;
        double leftPower;
        double rightPower;
        double maxPower;
        double dt;
        do {
            clicksRemaining = (int) (targetClicks - Math.abs(driveTrain.getCurrentPos()));
            inchesRemaining = clicksRemaining / CLICKS_PER_CM;
            power = DIRECTION.value * speed * inchesRemaining * -KP_STRAIGHT;
            power = Range.clip(power, -1.0, +1.0);
            dt = loopTimer.milliseconds();
            loopTimer.reset();
            angularError = imu.adjustAngle(targetAngle - imu.getHeading());
            angularIntegral = angularIntegral + angularError * dt;
            angularDerivative = (angularError - prevAngularError) / dt;
            prevAngularError = angularError;
            powerAdjustment = (.2 * power + .01) * angularError + KI_STRAIGHT * angularIntegral + KD_STRAIGHT * angularDerivative;
            powerAdjustment = Range.clip(powerAdjustment, -1.0, +1.0);
            powerAdjustment *= DIRECTION.value;
            leftPower = power - powerAdjustment;
            rightPower = power + powerAdjustment;
            maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxPower > 1.0) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }
            driveTrain.setPower(leftPower, rightPower);
        } while (opModeIsActive() && (inchesRemaining > 0.5 || Math.abs(angularError) > 0.5) && !timeoutTimer.elapsedTime(timeOut, MasqClock.Resolution.SECONDS));
        driveTrain.stopDriving();
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
    public void drive(int distance) {drive(distance, 0.5);}

    public void runToPosition(int distance, Direction direction, double speed, double timeOut, int sleepTime) {
        driveTrain.setDistance(distance);
        driveTrain.runToPosition(direction, speed, timeOut);
        sleep(sleepTime);
    }
    public void runToPosition(int distance, Direction direction, double speed, double timeOut) {
        runToPosition(distance, direction, speed, timeOut, DEFAULT_SLEEP_TIME);
    }
    public void runToPosition(int distance, Direction direction, double speed) {
        runToPosition(distance, direction, speed, DEFAULT_TIMEOUT);
    }
    public void runToPosition(int distance, Direction direction) {
        runToPosition(distance, direction, 0.7);
    }
    public void runToPosition(int distance) {
        runToPosition(distance, Direction.FORWARD);
    }

    public void turn(int angle, Direction DIRECTION, double timeOut, int sleepTime, double kp, double ki, double kd) {
        double targetAngle = imu.adjustAngle(imu.getHeading() + (DIRECTION.value * angle));
        double acceptableError = 0.5;
        double currentError = 1;
        double prevError = 0;
        double integral = 0;
        double newPower;
        double previousTime = 0;
        timeoutClock.reset();
        while (opModeIsActive() && (imu.adjustAngle(Math.abs(currentError)) > acceptableError)
                && !timeoutClock.elapsedTime(timeOut, MasqClock.Resolution.SECONDS)) {
            double tChange = System.nanoTime() - previousTime;
            previousTime = System.nanoTime();
            tChange = tChange / 1e9;
            double imuVAL = imu.getHeading();
            currentError = imu.adjustAngle(targetAngle - imuVAL);
            integral += currentError * ID;
            double errorkp = currentError * kp;
            double integralki = integral * ki * tChange;
            double dervitive = (currentError - prevError) / tChange;
            double dervitivekd = dervitive * kd;
            newPower = (errorkp + integralki + dervitivekd);
            newPower *= color;
            if (Math.abs(newPower) > 1.0) {newPower /= newPower;}
            driveTrain.setPower(newPower, -newPower);
            prevError = currentError;
            this.angleLeftCover = currentError;
            DashBoard.getDash().create("TargetAngle", targetAngle);
            DashBoard.getDash().create("Heading", imuVAL);
            DashBoard.getDash().create("AngleLeftToCover", currentError);
            DashBoard.getDash().update();
        }
        driveTrain.setPower(0,0);
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
    public void turn(int angle, Direction DIRECTION, double timeout)  {
        turn(angle, DIRECTION, timeout, DEFAULT_SLEEP_TIME);
    }
    public void turn(int angle, Direction DIRECTION)  {
        turn(angle, DIRECTION, DEFAULT_TIMEOUT);
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

    public void move(boolean control, MasqMotor motor, double power){
        if (control){
            motor.setPower(power);
        }
        else {
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
    public void move(boolean control, MasqServo servo, double position, double zeroPosition){
        if (control){
            servo.setPosition(position);
        }
        else {
            servo.setPosition(zeroPosition);
        }
    }
    public void move(boolean control, MasqServo servo, double position){
        if (control){
            servo.setPosition(position);
        }
        else {
            servo.setPosition(servo.getZero());
        }
    }
    public void move(boolean control, boolean backwordControl, MasqServo zero, double position, double zeroPosition){
        if (control){
            zero.setPosition(position);
        }
        else if (backwordControl){
            zero.setPosition(zeroPosition);
        }
    }
    public void move(boolean control, boolean backwordControl, MasqServo zero, double position){
        if (control){
            zero.setPosition(position);
        }
        else if (backwordControl){
            zero.setPosition(zero.getZero());
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

    public void NFS(MasqController c){
        float move = -c.left_stick_x();
        float turn = -c.right_stick_y();
        double left = move - turn;
        double right = move + turn;
        if(left > 1.0) {
            left /= left;
            right /= left;
            driveTrain.setPowerLeft(-left);
            driveTrain.setPowerRight(-right);
        }
        else if (right > 1.0) {
            left /= right;
            right /= right;
            driveTrain.setPowerLeft(-left);
            driveTrain.setPowerRight(-right);
        }
        else {
            driveTrain.setPowerLeft(-left);
            driveTrain.setPowerRight(-right);
        }
        voltageSensor.update();
    }
    public void MECH(MasqController c){
        double angle;
        double x = c.left_stick_y();
        double y = -c.left_stick_x();
        if (x != 0) {
            angle = Math.atan(y/x);
        }
        else {
            angle = 0;
        }
        if (x < 0 && y > 0) {
            angle = angle + Math.PI;
        }
        else if (x < 0 && y <= 0) {
            angle = angle + Math.PI;
        }
        else if (x > 0 && y < 0) {
            angle = angle + (2*Math.PI);
        }
        else if (x == 0 && y > 0 ) {
            angle = Math.PI/2;
        }
        else if (x == 0 && y < 0 ) {
            angle = (3 * Math.PI) / 2;
        }
        double speedMagnitude = Math.hypot(x, y);
        double frontLeft = -(Math.sin(angle + (Math.PI/4))) * speedMagnitude + c.right_stick_x();
        double backLeft = -(Math.cos(angle + (Math.PI/4))) * speedMagnitude + c.right_stick_x();
        double frontRight = (Math.cos(angle + (Math.PI/4))) * speedMagnitude + c.right_stick_x();
        double backRight = (Math.sin(angle + (Math.PI/4))) * speedMagnitude + c.right_stick_x();

        double driveScaleFactor = Math.abs(Math.max(
                Math.max(frontLeft, frontRight),
                Math.max(backLeft, backRight)))
                != 0 ? Math.abs(Math.max(
                Math.max(frontLeft, frontRight),
                Math.max(backLeft, backRight))) : 1
        ;
        frontLeft /= driveScaleFactor;
        frontRight /= driveScaleFactor;
        backLeft /= driveScaleFactor;
        backRight /= driveScaleFactor;
        driveTrain.leftDrive.motor1.setPower(frontLeft);
        driveTrain.leftDrive.motor2.setPower(backLeft);
        driveTrain.rightDrive.motor1.setPower(frontRight);
        driveTrain.rightDrive.motor2.setPower(backRight);
        voltageSensor.update();
    }
    public void TANK(MasqController c){
        driveTrain.rightDrive.setPower(c.right_stick_y());
        driveTrain.leftDrive.setPower(c.left_stick_x());
        voltageSensor.update();
    }

    public void vuforiaInit(){
        vuforia.setPositionOne(0,500,0);
        vuforia.setOrientationOne(MasqVuforia.Facing.RIGHT);
        vuforia.setPositionTwo(0,500,0);
        vuforia.setOrientationTwo(MasqVuforia.Facing.RIGHT);
        vuforia.setPositionThree(0,500,0);
        vuforia.setOrientationThree(MasqVuforia.Facing.RIGHT);
        vuforia.init();
    }
    public String getTrackable(){
            String v = null;
            if (vuforia.isSeen(Targets.TARGET_ONE))
                v =  "LEFT";
            if (vuforia.isSeen(Targets.TARGET_TWO))
                v =  "CENTER";
            if (vuforia.isSeen(Targets.TARGET_THREE))
                v =  "RIGHT";
            return v;
    }

    public int getDelta (double inital, Direction direction) {
        return (int) (inital- (imu.getHeading() * direction.value));
    }
    public double getVoltage() {
        return voltageSensor.getVoltage();
    }
    public void changeI2C(MasqHardware h, int newAd){

    }
    public void sleep(int time) {
        try {
            Thread.sleep((long) time);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
    public void sleep(double time) {
        try {
            Thread.sleep((long) time);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
    public void sleep() {
        sleep(DEFAULT_SLEEP_TIME);
    }
    public double getDelay(){return FtcRobotControllerActivity.getDelay();}
    private double scaleInput(double d)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };
        int index = (int) (d * 16.0);
        index = Math.abs(index);
        if (index > 16) {index = 16;}
        double dScale;
        if (d < 0) {dScale = -scaleArray[index];}
        else {dScale = scaleArray[index];}
        return dScale;
    }

}
