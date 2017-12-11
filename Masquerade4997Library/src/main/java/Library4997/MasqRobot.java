package Library4997;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import Library4997.MasqExternal.MasqExternal;
import Library4997.MasqExternal.MasqSensor;
import Library4997.MasqExternal.PID_CONSTANTS;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqMotors.MasqMotorSystem;
import Library4997.MasqMotors.MasqTankDrive;
import Library4997.MasqOpenCV.MasqOpenCV;
import Library4997.MasqSensors.MasqAdafruitIMU;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqSensors.MasqColorSensor;
import Library4997.MasqSensors.MasqLimitSwitch;
import Library4997.MasqSensors.MasqMatiboxUltraSensor;
import Library4997.MasqSensors.MasqREVColorSensor;
import Library4997.MasqSensors.MasqTouchSensor;
import Library4997.MasqSensors.MasqVoltageSensor;
import Library4997.MasqSensors.MasqVuforiaBeta;
import Library4997.MasqServos.MasqCRServo;
import Library4997.MasqServos.MasqCRServoSystem;
import Library4997.MasqServos.MasqServo;
import Library4997.MasqServos.MasqServoSystem;
import Library4997.MasqWrappers.DashBoard;
import Library4997.MasqExternal.Direction;
import Library4997.MasqWrappers.MasqController;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * MasqRobot--> Contains all hardware and methods to runLinearOpMode the robot.
 */
//TODO make MasqRobot abstract to support multiple copies of a robot, for test bot, main bot, so forth
public class MasqRobot implements PID_CONSTANTS {
    private static MasqLinearOpMode masqLinearOpMode;
    public MasqRobot (MasqLinearOpMode linearOpMode) {this.masqLinearOpMode = linearOpMode;}
    public MasqRobot () {}
    private static MasqRobot instance;
    public static MasqRobot getInstance (MasqLinearOpMode linearOpModeInstance) {
        if (instance==null) instance = new MasqRobot(linearOpModeInstance);
        return instance;
    }
    public MasqTankDrive driveTrain;
    public MasqMotor lift, relicLift;
    public MasqAdafruitIMU imu;
    public MasqCRServoSystem bottomIntake;
    public MasqServo blueRotator, redRotator;
    public MasqREVColorSensor jewelColorRed, jewelColorBlue;
    public MasqCRServo relicAdjuster;
    private MasqServo rightBottom, leftBottom;
    public MasqVoltageSensor voltageSensor;
    public MasqLimitSwitch bottomLimit, topLimit;
    public MasqServo jewelArmBlue, jewelArmRed, relicGripper;
    public MasqServoSystem glyphSystemBottom, glyphSystemTop;
    public MasqVuforiaBeta vuforia;
    public MasqMatiboxUltraSensor matiboxUltraSensor;
    //TODO GET MasqColorSensorV2 up.
    //public MasqMRColorSensor jewelColor;
    HardwareMap hardwareMap;
    private DashBoard dash;
    public MasqOpenCV openCV;
    public void mapHardware(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        dash = DashBoard.getDash();
        vuforia = new MasqVuforiaBeta();
        openCV = new MasqOpenCV();
        topLimit = new MasqLimitSwitch("topLimit", this.hardwareMap);
        blueRotator = new MasqServo("blueRotator", this.hardwareMap);
        redRotator = new MasqServo("redRotator", this.hardwareMap);
        bottomLimit = new MasqLimitSwitch("bottomLimit", this.hardwareMap);
        matiboxUltraSensor = new MasqMatiboxUltraSensor("ultra", this.hardwareMap);
        lift = new MasqMotor("lift", DcMotor.Direction.REVERSE, this.hardwareMap);
        driveTrain = new MasqTankDrive(this.hardwareMap);
        bottomIntake = new MasqCRServoSystem("leftBottomIntake", CRServo.Direction.FORWARD, "rightBottomIntake", CRServo.Direction.REVERSE, this.hardwareMap);
        relicAdjuster = new MasqCRServo("relicAdjuster", this.hardwareMap);
        rightBottom = new MasqServo("rightGlyphBottom", Servo.Direction.REVERSE, this.hardwareMap);
        leftBottom = new MasqServo("leftGlyphBottom", Servo.Direction.FORWARD, this.hardwareMap);
        glyphSystemTop = new MasqServoSystem("leftGlyphTop", Servo.Direction.FORWARD, "rightGlyphTop", Servo.Direction.REVERSE, this.hardwareMap);
        glyphSystemBottom = new MasqServoSystem(leftBottom, rightBottom);
        imu = new MasqAdafruitIMU("imu", this.hardwareMap);
        voltageSensor = new MasqVoltageSensor(this.hardwareMap);
        jewelArmBlue = new MasqServo("jewelArmBlue", this.hardwareMap);
        jewelArmRed = new MasqServo("jewelArmRed", this.hardwareMap);
        jewelColorRed = new MasqREVColorSensor("jewelColorRed", this.hardwareMap);
        jewelColorBlue = new MasqREVColorSensor("jewelColorBlue", this.hardwareMap);
        relicGripper = new MasqServo("relicGripper", this.hardwareMap);
        relicLift = new MasqMotor("relicLift", this.hardwareMap);
        createLimits();
    }

    private MasqClock timeoutClock = new MasqClock();
    public double angleLeftCover = 0;
    private double color = 1;

    public enum AllianceColor {
        BLUE (-1.0),
        RED (+1.0);
        public final double color;
        AllianceColor (double color) {this.color = color;}
    }
    public enum Targets {
        T1 ("T1");
        public final String value;
        Targets(String value) {this.value = value;}
    }
    public void setAllianceColor(AllianceColor allianceColor){this.color = allianceColor.color;}
    public static boolean opModeIsActive() {return masqLinearOpMode.opModeIsActive();}
    public void drive(int distance, double speed, Direction DIRECTION, double timeOut, int sleepTime) {
        MasqClock timeoutTimer = new MasqClock();
        MasqClock loopTimer = new MasqClock();
        driveTrain.resetEncoders();
        double targetAngle = imu.getHeading();
        int targetClicks = (int)(distance * CLICKS_PER_INCH);
        int clicksRemaining;
        double inchesRemaining, angularError = imu.adjustAngle(targetAngle - imu.getHeading()),
                prevAngularError = angularError, angularIntegral = 0,
                angularDerivative, powerAdjustment, power, leftPower, rightPower, maxPower, timeChange;
        do {
            clicksRemaining = (int) (targetClicks - Math.abs(driveTrain.getCurrentPosition()));
            inchesRemaining = clicksRemaining / CLICKS_PER_INCH;
            power = DIRECTION.value * speed * inchesRemaining * MasqExternal.KP.DRIVE_ENCODER;
            power = Range.clip(power, -1.0, +1.0);
            timeChange = loopTimer.milliseconds();
            loopTimer.reset();
            angularError = imu.adjustAngle(targetAngle - imu.getHeading());
            angularIntegral = (angularIntegral + angularError) * timeChange;
            angularDerivative = (angularError - prevAngularError) / timeChange;
            prevAngularError = angularError;
            powerAdjustment = (MasqExternal.KP.DRIVE_ANGULAR * power + .01) * angularError + (MasqExternal.KI.DRIVE * angularIntegral) + (MasqExternal.KD.DRIVE * angularDerivative);
            powerAdjustment = Range.clip(powerAdjustment, -1.0, +1.0);
            powerAdjustment *= DIRECTION.value;
            leftPower = power + powerAdjustment;
            rightPower = power - powerAdjustment;
            maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxPower > 1.0) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }
            driveTrain.setPower(leftPower, rightPower);
            dash.create("LEFT POWER: ",leftPower);
            dash.create("RIGHT POWER: ",rightPower);
            dash.create("ERROR: ",angularError);
        } while (opModeIsActive() && (inchesRemaining > 0.5 || Math.abs(angularError) > 0.5) && !timeoutTimer.elapsedTime(timeOut, MasqClock.Resolution.SECONDS));
        driveTrain.stopDriving();
        sleep(sleepTime);
    }
    public void drive(int distance, double speed, Direction strafe, double timeOut) {
        drive(distance, speed, strafe, timeOut, MasqExternal.DEFAULT_SLEEP_TIME);
    }
    public void drive(int distance, double speed, Direction strafe) {
        drive(distance, speed, strafe, MasqExternal.DEFAULT_TIMEOUT);
    }
    public void drive(int distance, double speed){drive(distance, speed, Direction.FORWARD);}
    public void drive(int distance) {drive(distance, 0.5);}

    public void runToPosition(int distance, Direction direction, double speed, double timeOut, int sleepTime) {
        driveTrain.setDistance(distance);
        driveTrain.runToPosition(direction, speed, timeOut);
        sleep(sleepTime);
    }
    public void runToPosition(int distance, Direction direction, double speed, double timeOut) {
        runToPosition(distance, direction, speed, timeOut, MasqExternal.DEFAULT_SLEEP_TIME);
    }
    public void runToPosition(int distance, Direction direction, double speed) {
        runToPosition(distance, direction, speed, MasqExternal.DEFAULT_TIMEOUT);
    }
    public void runToPosition(int distance, Direction direction) {
        runToPosition(distance, direction, 0.7);
    }
    public void runToPosition(int distance) {runToPosition(distance, Direction.FORWARD);}

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
            integral += currentError * MasqExternal.ID.TURN;
            double errorkp = currentError * kp;
            double integralki = integral * ki * tChange;
            double dervitive = (currentError - prevError) / tChange;
            double dervitivekd = dervitive * kd;
            newPower = (errorkp + integralki + dervitivekd);
            newPower *= color;
            if (newPower >= 1) {newPower /= Math.abs(newPower);}
            driveTrain.setPower(newPower, -newPower);
            prevError = currentError;
            this.angleLeftCover = currentError;
            dash.create("TargetAngle", targetAngle);
            dash.create("Heading", imuVAL);
            dash.create("AngleLeftToCover", currentError);
            dash.update();
        }
        driveTrain.setPower(0,0);
        sleep(sleepTime);
    }
    public void turn(int angle, Direction DIRECTION, double timeOut, int sleepTime, double kp, double ki) {
        turn(angle, DIRECTION, timeOut, sleepTime, kp, ki, MasqExternal.KD.TURN);
    }
    public void turn(int angle, Direction DIRECTION, double timeOut, int sleepTime, double kp) {
        turn(angle, DIRECTION, timeOut, sleepTime, kp, MasqExternal.KI.TURN);
    }
    public void turn(int angle, Direction DIRECTION, double timeOut, int sleepTime) {
        turn(angle, DIRECTION, timeOut, sleepTime, MasqExternal.KP.TURN);
    }
    public void turn(int angle, Direction DIRECTION, double timeout)  {
        turn(angle, DIRECTION, timeout, MasqExternal.DEFAULT_SLEEP_TIME);
    }
    public void turn(int angle, Direction DIRECTION)  {turn(angle, DIRECTION, MasqExternal.DEFAULT_TIMEOUT);}

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
            dash.create("Heading", heading);
            dash.create("Blue Val", colorSensor.colorNumber());
        }
        driveTrain.stopDriving();
    }
    public void stopBlue (MasqColorSensor colorSensor, double power){
        stopBlue(colorSensor, power, Direction.BACKWARD);
    }
    public void stopBlue (MasqColorSensor colorSensor){stopBlue(colorSensor, 0.5);}

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
            dash.create("Heading", heading);
            dash.create("red Val", colorSensor.colorNumber());
        }
        driveTrain.stopDriving();
    }
    public void stopRed (MasqColorSensor colorSensor, double power){
        stopRed(colorSensor, power, Direction.BACKWARD);
    }
    public void stopRed (MasqColorSensor colorSensor){stopRed(colorSensor, 0.5);}

    public void stop(MasqSensor sensor, double power, Direction Direction) {
        MasqClock loopTimer = new MasqClock();
        driveTrain.resetEncoders();
        double targetAngle = imu.getHeading();
        double  angularError = imu.adjustAngle(targetAngle - imu.getHeading()),
                prevAngularError = angularError, angularIntegral = 0,
                angularDerivative, powerAdjustment, leftPower, rightPower, maxPower, timeChange;
        do {
            power = Range.clip(power, -1.0, +1.0);
            timeChange = loopTimer.milliseconds();
            loopTimer.reset();
            angularError = imu.adjustAngle(targetAngle - imu.getHeading());
            angularIntegral = angularIntegral + angularError * timeChange;
            angularDerivative = (angularError - prevAngularError) / timeChange;
            prevAngularError = angularError;
            powerAdjustment = (MasqExternal.KP.DRIVE_ANGULAR * power + .01) * angularError + MasqExternal.KI.DRIVE * angularIntegral + MasqExternal.KD.DRIVE * angularDerivative;
            powerAdjustment = Range.clip(powerAdjustment, -1.0, +1.0);
            powerAdjustment *= Direction.value;
            leftPower = power - powerAdjustment;
            rightPower = power + powerAdjustment;
            maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxPower > 1.0) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }
            driveTrain.setPower(leftPower, rightPower);
            dash.create("LEFT POWER: ",leftPower);
            dash.create("RIGHT POWER: ",rightPower);
            dash.create("ERROR: ",angularError);
        } while (opModeIsActive() && sensor.stop());
        driveTrain.stopDriving();
    }
    public void stop (MasqSensor sensor, double power){
        stop(sensor, power, Direction.BACKWARD);
    }
    public void stop (MasqSensor sensor){stop(sensor, 0.5);}
    public void NFS(MasqController c) {
        float move = c.leftStickY();
        float turn = c.rightStickX();
        double left = move - turn;
        double right = move + turn;
        left *= -.6;
        right *= -.6;
        if (c.leftBumper()) {
            left /= 2;
            right /= 2;
        }
        if(left > 1.0) {
            left /= left;
            right /= left;
            driveTrain.setPowerLeft(left);
            driveTrain.setPowerRight(right);
        }
        else if (right > 1.0) {
            left /= right;
            right /= right;
            driveTrain.setPowerLeft(left);
            driveTrain.setPowerRight(right);
        }
        else {
            driveTrain.setPowerLeft(left);
            driveTrain.setPowerRight(right);
        }
        voltageSensor.update();

    }
    public void MECH(MasqController c) {
        double x = -c.leftStickY();
        double y = c.leftStickX();
        double angle = Math.atan2(y, x);
        double adjustedAngle = angle + Math.PI/4;
        double multiplier = 1.4;
        double speedMagnitude = Math.hypot(x, y);
        double leftFront = (Math.sin(adjustedAngle) * speedMagnitude * multiplier) - c.rightStickX() * multiplier;
        double leftBack = (Math.cos(adjustedAngle) * speedMagnitude * multiplier) - c.rightStickX() * multiplier;
        double rightFront = (Math.cos(adjustedAngle) * speedMagnitude * multiplier) + c.rightStickX()* multiplier;
        double rightBack = (Math.sin(adjustedAngle) * speedMagnitude * multiplier) + c.rightStickX() * multiplier;
        double max = Math.max(Math.max(Math.abs(leftFront), Math.abs(leftBack)), Math.max(Math.abs(rightFront), Math.abs(rightBack)));
        if (max > 1) {
            leftFront /= max;
            leftBack /= max;
            rightFront /= max;
            rightBack /= max;
        }
        driveTrain.leftDrive.motor1.setPower(leftFront);
        driveTrain.leftDrive.motor2.setPower(leftBack);
        driveTrain.rightDrive.motor1.setPower(rightFront);
        driveTrain.rightDrive.motor2.setPower(rightBack);
        dash.create("LEFT FRONT: ", leftFront);
        dash.create("LEFT BACK: ", leftBack);
        dash.create("RIGHT FRONT: ", rightFront);
        dash.create("RIGHT BACK: ", rightBack);
        dash.create("LEFT FRONT POWER: ", driveTrain.leftDrive.motor1.getRate());
        dash.create("LEFT BACK POWER: ", driveTrain.leftDrive.motor2.getRate());
        dash.create("RIGHT FRONT POWER: ", driveTrain.rightDrive.motor1.getRate());
        dash.create("RIGHT BACK POWER: ", driveTrain.rightDrive.motor2.getRate());
        dash.update();
    }
    public void TANK(MasqController c){
        double left = c.leftStickX();
        double right = c.rightStickY();
        double leftRate = driveTrain.leftDrive.getRate();
        double rightRate = driveTrain.rightDrive.getRate();
        double maxRate = Math.max(Math.abs(leftRate/left), Math.abs(rightRate/right));
        leftRate /= maxRate;
        rightRate /= maxRate;
        double leftError =  left - leftRate;
        double rightError = right - rightRate;
        driveTrain.rightDrive.setPower(right - (rightError * KP_TELE));
        driveTrain.leftDrive.setPower(left - (leftError * KP_TELE));
        voltageSensor.update();
    }

    public int getDelta (double initial, Direction direction) {
        return (int) (initial- (imu.getHeading() * direction.value));
    }
    public double getVoltage() {
        return voltageSensor.getVoltage();
    }
    public double getDelay() {return FtcRobotControllerActivity.getDelay();}

    public void waitForVuMark() {
        timeoutClock.reset();
        while (MasqExternal.VuMark.isUnKnown(vuforia.getVuMark()) &&
                !timeoutClock.elapsedTime(5, MasqClock.Resolution.SECONDS)){
            dash.create(vuforia.getVuMark());
            dash.update();
        }
        dash.create(vuforia.getVuMark());
        dash.update();
    }
    public void initializeTeleop(){
        driveTrain.setKp(MasqExternal.KP.MOTOR_TELEOP);
        driveTrain.setKi(MasqExternal.KI.MOTOR_TELEOP);
        driveTrain.setKp(MasqExternal.KD.MOTOR_TELEOP);
    }
    public void initializeAutonomous(){
        driveTrain.setKp(MasqExternal.KP.MOTOR_AUTONOMOUS);
        driveTrain.setKi(MasqExternal.KI.MOTOR_AUTONOMOUS);
        driveTrain.setKp(MasqExternal.KD.MOTOR_AUTONOMOUS);
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
    public void sleep() {sleep(MasqExternal.DEFAULT_SLEEP_TIME);}

    public void initializeServos() {
        glyphSystemTop.setPosition(1);
        glyphSystemBottom.setPosition(0);
        jewelArmBlue.setPosition(0);
        jewelArmRed.setPosition(0.6);
    }
    public void createLimits () {
        //bottomIntake.setLimit(bottomLimit);
    }
}
