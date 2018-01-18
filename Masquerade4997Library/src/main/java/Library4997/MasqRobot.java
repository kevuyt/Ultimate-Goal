package Library4997;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import Library4997.MasqExternal.MasqExternal;
import Library4997.MasqExternal.MasqSensor;
import Library4997.MasqExternal.MasqSerializer;
import Library4997.MasqExternal.PID_CONSTANTS;
import Library4997.MasqExternal.Strafe;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqMotors.MasqMotorSystem;
import Library4997.MasqMotors.MasqTankDrive;
import Library4997.MasqOpenCV.MasqOpenCV;
import Library4997.MasqSensors.MasqAdafruitIMUv2;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqSensors.MasqColorSensor;
import Library4997.MasqSensors.MasqREVColorSensor;
import Library4997.MasqSensors.MasqVoltageSensor;
import Library4997.MasqSensors.MasqVuforiaBeta;
import Library4997.MasqServos.MasqCRServo;
import Library4997.MasqServos.MasqServo;
import Library4997.MasqServos.MasqServoSystem;
import Library4997.MasqWrappers.DashBoard;
import Library4997.MasqExternal.Direction;
import Library4997.MasqWrappers.MasqController;

/**
 * MasqRobot--> Contains all hardware and methods to runLinearOpMode the robot.
 */
//TODO make MasqRobot abstract to support multiple copies of a robot, for test bot, main bot, so forth
public class MasqRobot implements PID_CONSTANTS {
    public MasqRobot () {}
    public MasqTankDrive driveTrain;
    public MasqMotorSystem intake;
    public MasqMotor lift, relicLift;
    public MasqAdafruitIMUv2 imu;
    public MasqServo blueRotator, redRotator;
    public MasqREVColorSensor jewelColorRed, jewelColorBlue;
    public MasqServoSystem flipper;
    public MasqCRServo relicAdjuster;
    public MasqVoltageSensor voltageSensor;
    public MasqSerializer serializer;
    public MasqServo jewelArmBlue, jewelArmRed, relicGripper;
    public MasqVuforiaBeta vuforia;
    private double acceptableDriveError = .5;
    //TODO GET MasqColorSensorV2 up.
    //public MasqMRColorSensor jewelColor;
    HardwareMap hardwareMap;
    private DashBoard dash;
    public MasqOpenCV openCV;
    public void mapHardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        dash = DashBoard.getDash();
        vuforia = new MasqVuforiaBeta();
        intake = new MasqMotorSystem("leftIntake", DcMotor.Direction.REVERSE, "rightIntake", DcMotor.Direction.FORWARD, "INTAKE", this.hardwareMap);
        voltageSensor = new MasqVoltageSensor(this.hardwareMap);
        openCV = new MasqOpenCV();
        serializer = new MasqSerializer();
        flipper = new MasqServoSystem("flipLeft", Servo.Direction.FORWARD, "flipRight", Servo.Direction.REVERSE, this.hardwareMap);
        blueRotator = new MasqServo("blueRotator", this.hardwareMap);
        redRotator = new MasqServo("redRotator", this.hardwareMap);
        lift = new MasqMotor("lift", DcMotor.Direction.REVERSE, this.hardwareMap);
        driveTrain = new MasqTankDrive(this.hardwareMap);
        relicAdjuster = new MasqCRServo("relicAdjuster", this.hardwareMap);
        imu = new MasqAdafruitIMUv2("imuHubOne", this.hardwareMap);
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
    public static boolean opModeIsActive() {return MasqExternal.opModeIsActive();}
    public void drive(int distance, double speed, Direction DIRECTION, double timeOut, int sleepTime) {
        serializer.createFile(new String[]{"Clicks Remaining", "Power", "Angular Error", "Angular Intergral", "Angular Derivative", "Left Power", "Right Power", "Power Adjustment" }, "DRIVEPID");
        driveTrain.setClosedLoop(true);
        MasqClock timeoutTimer = new MasqClock();
        MasqClock loopTimer = new MasqClock();
        driveTrain.resetEncoders();
        double targetAngle = imu.getHeading();
        int targetClicks = (int)(distance * MasqExternal.CLICKS_PER_INCH);
        int clicksRemaining;
        double inchesRemaining, angularError = imu.adjustAngle(targetAngle - imu.getHeading()),
                prevAngularError = angularError, angularIntegral = 0,
                angularDerivative, powerAdjustment, power, leftPower, rightPower, maxPower, timeChange;
        do {
            clicksRemaining = (int) (targetClicks - Math.abs(driveTrain.getCurrentPosition()));
            inchesRemaining = clicksRemaining / MasqExternal.CLICKS_PER_INCH;
            power = DIRECTION.value * (1 - (clicksRemaining / targetClicks)) * speed * MasqExternal.KP.DRIVE_ENCODER;
            power = Range.clip(power, -1.0, +1.0);
            timeChange = loopTimer.milliseconds();
            loopTimer.reset();
            angularError = imu.adjustAngle(targetAngle - imu.getHeading());
            angularIntegral = (angularIntegral + angularError) * timeChange;
            angularDerivative = (angularError - prevAngularError) / timeChange;
            prevAngularError = angularError;
            powerAdjustment = (MasqExternal.KP.DRIVE_ANGULAR * power) * angularError + (MasqExternal.KI.DRIVE * angularIntegral) + (MasqExternal.KD.DRIVE * angularDerivative);
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
            serializer.writeData(new Object[]{clicksRemaining, power, angularError, angularIntegral, angularDerivative, leftPower, rightPower, powerAdjustment});
            dash.create("LEFT POWER: ",leftPower);
            dash.create("RIGHT POWER: ",rightPower);
            dash.create("ERROR: ",angularError);
        } while (opModeIsActive() && (inchesRemaining > acceptableDriveError || Math.abs(angularError) > 0.5) && !timeoutTimer.elapsedTime(timeOut, MasqClock.Resolution.SECONDS));
        serializer.close();
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
        serializer.createFile(new String[]{"Error", "Proprtional", "Intergral", "Derivitive", "Left Power", " Right Power"}, "TURNPID");
        driveTrain.setClosedLoop(false);
        double targetAngle = imu.adjustAngle(imu.getHeading() + (DIRECTION.value * angle));
        double acceptableError = .5;
        double turnPower = .4;
        double currentError = imu.adjustAngle(targetAngle - imu.getHeading());
        double prevError = 0;
        double integral = 0;
        double derivative;
        double newPower;
        double previousTime = 0;
        timeoutClock.reset();
        while (opModeIsActive() && (imu.adjustAngle(Math.abs(currentError)) > acceptableError)
                && !timeoutClock.elapsedTime(timeOut, MasqClock.Resolution.SECONDS)) {
            double tChange = System.nanoTime() - previousTime;
            previousTime = System.nanoTime();
            tChange = tChange / 1e9;
            currentError = imu.adjustAngle(targetAngle - imu.getHeading());
            integral += currentError * tChange;
            derivative = (currentError - prevError) / tChange;
            double errorkp = currentError * kp;
            double integralki = integral * ki;
            double dervitivekd = derivative * kd;
            newPower = (errorkp + integralki + dervitivekd);
            if (Math.abs(newPower) >= 1) {newPower /= Math.abs(newPower);}
            driveTrain.setPower(-newPower * turnPower, newPower * turnPower);
            prevError = currentError;
            this.angleLeftCover = currentError;
            serializer.writeData(new Object[]{currentError, errorkp, integralki, dervitivekd, -newPower, newPower});
            dash.create("TargetAngle", targetAngle);
            dash.create("Heading", imu.getHeading());
            dash.create("AngleLeftToCover", currentError);
            dash.update();
        }
        serializer.close();
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

    public void stop(MasqSensor sensor, double speed, Direction Direction, int amount) {
        int currentTimes = 0;
        MasqClock loopTimer = new MasqClock();
        driveTrain.resetEncoders();
        double targetAngle = imu.getHeading();
        double  angularError = imu.adjustAngle(targetAngle - imu.getHeading()),
                prevAngularError = angularError, angularIntegral = 0,
                angularDerivative, powerAdjustment, leftPower, rightPower, maxPower, timeChange, power;
        do {
            power = Direction.value * (1 - (currentTimes / amount)) * speed;
            power = Range.clip(power, -1.0, +1.0);
            timeChange = loopTimer.milliseconds();
            loopTimer.reset();
            angularError = imu.adjustAngle(targetAngle - imu.getHeading());
            angularIntegral = angularIntegral + angularError * timeChange;
            angularDerivative = (angularError - prevAngularError) / timeChange;
            prevAngularError = angularError;
            powerAdjustment = (MasqExternal.KP.DRIVE_ANGULAR * power + .01) * angularError +
                    MasqExternal.KI.DRIVE * angularIntegral + MasqExternal.KD.DRIVE * angularDerivative;
            powerAdjustment = Range.clip(powerAdjustment, -1.0, +1.0);
            powerAdjustment *= Direction.value;
            leftPower = power - powerAdjustment;
            rightPower = power + powerAdjustment;
            maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxPower > 1.0) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }
            if (sensor.stop()) currentTimes++;
            driveTrain.setPower(leftPower, rightPower);
            dash.create("LEFT POWER: ",leftPower);
            dash.create("RIGHT POWER: ",rightPower);
            dash.create("ERROR: ",angularError);
        } while (opModeIsActive() && currentTimes < amount);
        driveTrain.stopDriving();
    }
    public void stop (MasqSensor sensor, double power, Direction direction) {stop(sensor, power, direction, 1);}
    public void stop(MasqSensor sensor, double power){
        stop(sensor, power, Direction.BACKWARD);
    }
    public void stop (MasqSensor sensor){stop(sensor, 0.5);}

    public void strafe(int distance, Strafe direction, double speed, double timeOut, double sleepTime) {
        driveTrain.setClosedLoop(true);
        driveTrain.resetEncoders();
        int targetClicks = (int)(distance * MasqExternal.CLICKS_PER_INCH);
        int clicksRemaining = targetClicks;
        double power, intergral = 0, deriviteve, timeChange, previousTime = 0, previousClicksRemaining = clicksRemaining;
        MasqClock timeoutTimer = new MasqClock();
        do {
            timeChange = System.nanoTime() - previousTime;
            previousTime = System.nanoTime();
            timeChange /= 1e9;
            clicksRemaining = (int) (targetClicks - Math.abs(driveTrain.getCurrentPosition()));
            intergral += clicksRemaining * timeChange;
            deriviteve = (clicksRemaining - previousClicksRemaining) / timeChange;
            power = (clicksRemaining / targetClicks) * speed;
            power += (intergral * MasqExternal.KI.DRIVE) + (deriviteve * MasqExternal.KD.DRIVE);
            driveTrain.leftDrive.motor1.setPower(power * direction.value[0]);
            driveTrain.rightDrive.motor1.setPower(power * direction.value[1]);
            driveTrain.rightDrive.motor2.setPower(power * direction.value[2]);
            driveTrain.leftDrive.motor2.setPower(power * direction.value[3]);
        } while (opModeIsActive() && clicksRemaining > 0.5 && !timeoutTimer.elapsedTime(timeOut, MasqClock.Resolution.SECONDS));
        sleep(sleepTime);
    }
    public void strafe(int distance, Strafe direction, double speed, double timeOut) {
        strafe(distance, direction, speed, timeOut, MasqExternal.DEFAULT_SLEEP_TIME);
    }
    public void strafe(int distance, Strafe direction, double speed) {
        strafe(distance, direction, speed, MasqExternal.DEFAULT_TIMEOUT, MasqExternal.DEFAULT_SLEEP_TIME);
    }
    public void strafe(int distance, Strafe direction) {
        strafe(distance, direction, 0.7, MasqExternal.DEFAULT_TIMEOUT, MasqExternal.DEFAULT_SLEEP_TIME);
    }

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
        double leftFront = (Math.sin(adjustedAngle) * speedMagnitude * multiplier) + c.rightStickX() * multiplier;
        double leftBack = (Math.cos(adjustedAngle) * speedMagnitude * multiplier) + c.rightStickX()  * multiplier;
        double rightFront = (Math.cos(adjustedAngle) * speedMagnitude * multiplier) - c.rightStickX() * multiplier;
        double rightBack = (Math.sin(adjustedAngle) * speedMagnitude * multiplier) - c.rightStickX() * multiplier;
        double max = Math.max(Math.max(Math.abs(leftFront), Math.abs(leftBack)), Math.max(Math.abs(rightFront), Math.abs(rightBack)));
        if (max > 1) {
            leftFront /= max;
            leftBack /= max;
            rightFront /= max;
            rightBack /= max;
        }
        if (c.leftTriggerPressed()) {
            leftFront /= 3;
            leftBack /= 3;
            rightFront /= 3;
            rightBack /= 3;
        }
        driveTrain.leftDrive.motor1.setPower(leftFront);
        driveTrain.leftDrive.motor2.setPower(leftBack);
        driveTrain.rightDrive.motor1.setPower(rightFront);
        driveTrain.rightDrive.motor2.setPower(rightBack);
        dash.create("FRONT LEFT: ", leftFront);
        dash.create("FRONT RIGHT: ", rightFront);
        dash.create("BACK RIGHT: ", rightBack);
        dash.create("BACK LEFT: ", leftBack);
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
        driveTrain.rightDrive.setPower(right - (rightError * MasqExternal.KP.MOTOR_TELEOP));
        driveTrain.leftDrive.setPower(left - (leftError *  MasqExternal.KP.MOTOR_TELEOP));
        voltageSensor.update();
    }

    public int getDelta (double initial, Direction direction) {
        return (int) (initial- (imu.getHeading() * direction.value));
    }
    public double getVoltage() {return voltageSensor.getVoltage();}
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

    public void setAcceptableDriveError(double acceptableDriveError) {
        this.acceptableDriveError = acceptableDriveError;
    }
    public void initializeServos() {
        jewelArmBlue.setPosition(0);
        jewelArmRed.setPosition(0);
    }
    public void createLimits () {
        //bottomIntake.setLimit(bottomLimit);
    }
}
