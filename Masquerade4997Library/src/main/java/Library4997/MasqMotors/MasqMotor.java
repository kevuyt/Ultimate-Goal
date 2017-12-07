package Library4997.MasqMotors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqExternal.MasqExternal;
import Library4997.MasqExternal.MasqHardware;
import Library4997.MasqRobot;
import Library4997.MasqExternal.Direction;
import Library4997.MasqExternal.PID_CONSTANTS;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqSensors.MasqLimitSwitch;

/**
 * This is a custom motor that includes stall detection and telemetry
 */
public class MasqMotor implements PID_CONSTANTS, MasqHardware {
    private DcMotor motor;
    private String nameMotor;
    private int direction = 1;
    private double kp = 0.004, ki = 0, kd = 0;
    private double holdKp = 0.0002;
    private boolean closedLoop = true;
    private boolean holdPositionMode = false;
    private double targetPosition = 0;
    private double prevPos = 0;
    private double previousTime = 0;
    private double destination = 0;
    public double currentPower;
    private double currentMax, currentMin;
    private double currentZero;
    private double holdItergral = 0;
    private double holdDerivitive = 0;
    private double holdPreviousError = 0;
    private double rpmIntegral = 0;
    private double rpmDerivative = 0;
    private double rpmPreviousError = 0;
    private double currentPosition = 0, zeroEncoderPosition = 0, prevRate = 0;
    private double minPosition, maxPosition;
    private boolean servoMode;
    private boolean limitDetection, positionDetection, halfDetectionMin, halfDetectionMax;
    private MasqClock timeoutTimer = new MasqClock();
    private MasqLimitSwitch minLim, maxLim = null;
    public MasqMotor(String name, HardwareMap hardwareMap){
        limitDetection = positionDetection = false;
        this.nameMotor = name;
        motor = hardwareMap.get(DcMotor.class, name);
    }
    public MasqMotor(String name, DcMotor.Direction direction, HardwareMap hardwareMap) {
        limitDetection = positionDetection = false;
        if (direction == DcMotor.Direction.REVERSE) this.direction = 1;
        this.nameMotor = name;
        motor = hardwareMap.dcMotor.get(name);
        motor.setDirection(direction);
    }
    public MasqMotor setLimits(MasqLimitSwitch min, MasqLimitSwitch max){
        maxLim = max; minLim = min;
        limitDetection = true;
        return this;
    }
    public MasqMotor setLimit(MasqLimitSwitch min){
        minLim = min; maxLim = null;
        limitDetection = true;
        return this;
    }
    public MasqMotor setPositionLimits (double min, double max) {
        minPosition = min; maxPosition = max;
        positionDetection = true;
        return this;
    }
    public MasqMotor setHalfLimits(MasqLimitSwitch min, double max){
        maxPosition = max;
        minLim = min; halfDetectionMin = true;
        return this;
    }
    public MasqMotor setHalfLimits(double min, MasqLimitSwitch max){
        minPosition = min;
        maxLim = max; halfDetectionMax = true;
        return this;
    }
    public MasqMotor setPositionLimit (double min) {
        minPosition = min;
        positionDetection = true;
        return this;
    }
    public void runWithoutEncoders () {motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);}
    public void resetEncoder() {
        zeroEncoderPosition = motor.getCurrentPosition();
        currentPosition = 0;

    }
    public void setLazy() {
        holdPositionMode = false;
    }
    public void setStrong() {
        holdPositionMode = true;
        targetPosition = getCurrentPosition();
    }
    public void setPower (double power) {
        double motorPower = findPower(power);
        if (limitDetection) {
            if (minLim != null && minLim.isPressed() && power < 0 ||
                    maxLim != null && maxLim.isPressed() && power > 0)
                motorPower = 0;
            else if (minLim != null && minLim.isPressed()
                    && power < 0 && maxLim == null)
                motorPower = 0;
        } else if (positionDetection) {
            if ((motor.getCurrentPosition() < minPosition && power < 0) ||
                    (motor.getCurrentPosition() > maxPosition && power > 0))
                motorPower = 0;
            else if (motor.getCurrentPosition() < minPosition && power < 0)
                motorPower = 0;
        } else if (halfDetectionMin) {
            if (minLim.isPressed()) {
                currentZero = motor.getCurrentPosition();
                currentMax = currentZero + maxPosition;
            }
            if (minLim != null && minLim.isPressed() && power < 0) motorPower = 0;
            else if (motor.getCurrentPosition() > currentMax && power > 0) motorPower = 0;
        } else if (halfDetectionMax) {
            if (maxLim.isPressed()) {
                currentZero = motor.getCurrentPosition();
                currentMin = currentZero - minPosition;
            }
            if (maxLim != null && maxLim.isPressed() && power >0) motorPower = 0;
            else if (motor.getCurrentPosition() < currentMin && power < 0) motorPower = 0;
        }
        currentPower = motorPower;
        motor.setPower(motorPower);
    }
    public void runUsingEncoder() {motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);}
    public void setDistance (double distance) {
        resetEncoder();
        destination = distance;
    }
    private boolean opModeIsActive() {
        return MasqRobot.getInstance(null).opModeIsActive();
    }
    public void runToPosition(Direction direction, double speed){
        timeoutTimer.reset();
        resetEncoder();
        int targetClicks = (int)(destination * CLICKS_PER_INCH);
        int clicksRemaining;
        double inchesRemaining;
        double power;
        do {
            clicksRemaining = (int) (targetClicks - Math.abs(getCurrentPosition()));
            inchesRemaining = clicksRemaining / CLICKS_PER_INCH;
            power = direction.value * speed * inchesRemaining * KP_STRAIGHT;
            setPower(power);
        } while (opModeIsActive() && inchesRemaining > 0.5 && timeoutTimer.elapsedTime(2, MasqClock.Resolution.SECONDS));
        setPower(0);
    }
    boolean isBusy () {
        return motor.isBusy();
    }
    public void setBreakMode () {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public double getCurrentPosition() {
        currentPosition = motor.getCurrentPosition() - zeroEncoderPosition;
        return currentPosition;
    }
    public double getPower() {return currentPower;}
    public double getRate () {
        double deltaPosition = getCurrentPosition() - prevPos;
        double tChange = System.nanoTime() - previousTime;
        previousTime = System.nanoTime();
        tChange = tChange / 1e9;
        prevPos = getCurrentPosition();
        double rate = deltaPosition / tChange;
        rate = (rate * 60) / MasqExternal.NEVERREST_40_TICKS_PER_ROTATION;
        if (rate != 0) return rate;
        else {
            prevRate = rate;
            return prevRate;
        }
    }
    public String getName() {
        return nameMotor;
    }
    public void setClosedLoop(boolean closedLoop) {this.closedLoop = closedLoop;}
    private double findPower(double power){
        if (holdPositionMode) {
            double tChange = (System.nanoTime() - previousTime) / 1e9;
            double error = targetPosition - getCurrentPosition();
            holdItergral += error * tChange;
            holdDerivitive = (error - holdPreviousError) / tChange;
            power = (direction * ((error * holdKp) +
                    (holdItergral * ki) + (holdDerivitive * kd)));
            holdPreviousError = error;
        }
        if (closedLoop) {
            double error, setRPM, currentRPM, motorPower;
            double tChange = System.nanoTime() - previousTime;
            tChange /= 1e9;
            setRPM = MasqExternal.NEVERREST_40_RPM * power;
            currentRPM = getRate();
            error = setRPM - currentRPM;
            rpmIntegral += error * tChange;
            rpmDerivative = (error - rpmPreviousError) / tChange;
            motorPower = (power) + (direction * ((error * kp) +
                    (rpmIntegral * ki) + (rpmDerivative * kd)));
            rpmPreviousError = error;
            return motorPower;
        }
        else return power;
    }

    public double getKp() {return kp;}
    public void setKp(double kp) {this.kp = kp;}
    public double getKi() {return ki;}
    public void setKi(double ki) {this.ki = ki;}
    public double getKd() {return kd;}
    public void setKd(double kd) {this.kd = kd;}
    public String[] getDash() {
        return new String[] {"Current Position" + Double.toString(getCurrentPosition())};
    }
}

