package Library4997.MasqMotors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import Library4997.MasqResources.MasqHardware;
import Library4997.MasqUtils;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqSensors.MasqEncoder;
import Library4997.MasqSensors.MasqLimitSwitch;

/**
 * This is a custom motor that includes stall detection and telemetry
 */
public class MasqMotor implements MasqHardware {
    private double minPower = 0;
    public DcMotor motor;
    private boolean stallDetection = false;
    private String nameMotor;
    private double targetPower;
    private boolean velocityControlState = false;
    private double kp = 0.001, ki = 0, kd = 0;
    public MasqEncoder encoder;
    private double prevPos = 0;
    private boolean stalled = false;
    private double previousTime = 0;
    private double motorPower;
    private double currentMax, currentMin;
    public double rpmIntegral = 0;
    public double rpmDerivative = 0;
    private double rpmPreviousError = 0;
    private int stalledRPMThreshold = 10;
    private boolean reversedEncoder = false;
    private Runnable
            stallAction,
            unStalledAction;
    private double minPosition, maxPosition;
    private boolean
            limitDetection,
            positionDetection,
            halfDetectionMin = false,
            halfDetectionMax = false,
            closedLoop = false;
    private MasqLimitSwitch minLim, maxLim = null;

    public MasqMotor(String name, MasqMotorModel model, HardwareMap hardwareMap) {
        limitDetection = positionDetection = false;
        this.nameMotor = name;
        motor = hardwareMap.get(DcMotor.class, name);
        encoder = new MasqEncoder(this, model);
    }
    public MasqMotor(String name, MasqMotorModel model, DcMotor.Direction direction, HardwareMap hardwareMap) {
        limitDetection = positionDetection = false;
        this.nameMotor = name;
        motor = hardwareMap.dcMotor.get(name);
        motor.setDirection(direction);
        encoder = new MasqEncoder(this, model);
    }

    public void setLimits(MasqLimitSwitch min, MasqLimitSwitch max){
        maxLim = max; minLim = min;
        limitDetection = true;
    }
    public MasqMotor setLimit(MasqLimitSwitch min) {
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

    public void runWithoutEncoder() {motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);}
    public void runUsingEncoder() {motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);}

    public void resetEncoder() {
        encoder.resetEncoder();
    }


    public void runToPosition(int inches, double speed){
        MasqClock clock = new MasqClock();
        resetEncoder();
        double clicks = -inches * encoder.getClicksPerInch();
        motor.setTargetPosition((int) clicks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setVelocity(speed);
        while (opModeIsActive() && motor.isBusy() &&
                clock.hasNotPassed(5, MasqClock.Resolution.SECONDS)) {MasqUtils.sleep(0.1);}
        setVelocity(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    boolean isBusy () {
        return motor.isBusy();
    }

    public double getCurrentPosition() {
        return encoder.getRelativePosition();
    }
    public double getAbsolutePosition () {
        if(reversedEncoder) return -motor.getCurrentPosition();
        return motor.getCurrentPosition();
    }
    public double getVelocity() {
        double deltaPosition = getCurrentPosition() - prevPos;
        previousTime = System.nanoTime();
        double tChange = (System.nanoTime() - previousTime) / 1e9;
        prevPos = getCurrentPosition();
        double rate = deltaPosition / tChange;
        rate = (rate * 60) / encoder.getClicksPerRotation();
        return rate;
    }

    public void setPower (double power) {
        power = Range.clip(power, -1, 1);
        motorPower = power;
        motor.setPower(power);
    }
    public void setVelocity(double power) {
        targetPower = power;
        motorPower = calculateVelocityCorrection(power);
        if (!closedLoop) motorPower = power;
        double currentZero;
        if (limitDetection) {
            if (minLim != null && minLim.isPressed() && power < 0 ||
                    maxLim != null && maxLim.isPressed() && power > 0)
                motorPower = 0;
            else if (minLim != null && minLim.isPressed()
                    && power < 0 && maxLim == null)
                motorPower = 0;
        }
        else if (positionDetection) {
            if ((motor.getCurrentPosition() < minPosition && power < 0) ||
                    (motor.getCurrentPosition() > maxPosition && power > 0))
                motorPower = 0;
            else if (motor.getCurrentPosition() < minPosition && power < 0)
                motorPower = 0;
        }
        else if (halfDetectionMin) {
            if (minLim.isPressed()) {
                currentZero = motor.getCurrentPosition();
                currentMax = currentZero + maxPosition;
            }
            if (minLim != null && minLim.isPressed() && power < 0) motorPower = 0;
            else if (motor.getCurrentPosition() > currentMax && power > 0) motorPower = 0;
        }
        else if (halfDetectionMax) {
            if (maxLim.isPressed()) {
                currentZero = motor.getCurrentPosition();
                currentMin = currentZero - minPosition;
            }
            if (maxLim != null && maxLim.isPressed() && power >0) motorPower = 0;
            else if (motor.getCurrentPosition() < currentMin && power < 0) motorPower = 0;
        }
        if (Math.abs(motorPower) < minPower && minPower != 0) motorPower = 0;
        motor.setPower(motorPower);
    }
    public  double calculateVelocityCorrection(double power) {
        double tChange = (System.nanoTime() - previousTime)/1e9;
        double error = (encoder.getRPM() * power) - getVelocity();
        rpmIntegral += error * tChange;
        rpmDerivative = (error - rpmPreviousError) / tChange;
        double p = error*kp;
        double i = rpmIntegral*ki;
        double d = rpmDerivative*kd;
        double motorPower = power + (p + i + d);
        rpmPreviousError = error;
        previousTime = System.nanoTime();
        return motorPower;
    }

    public void setVelocityControlState(boolean velocityControlState) {
        this.velocityControlState = velocityControlState;
    }
    public void startVelocityControl () {
        setVelocityControlState(true);
        Runnable velocityControl = () -> {
            while (opModeIsActive() && velocityControlState) {
                setVelocity(targetPower);
            }
        };
        Thread velocityThread = new Thread(velocityControl);
        velocityThread.start();
    }

    private boolean getStalled() {
        return Math.abs(getVelocity()) < stalledRPMThreshold;
    }
    public void setStalledAction(Runnable action) {
        stallAction = action;
    }
    public void setUnStalledAction(Runnable action) {
        unStalledAction = action;
    }
    public void setStallDetection(boolean bool) {stallDetection = bool;}
    private boolean getStallDetection () {return stallDetection;}
    public synchronized boolean isStalled() {
        return stalled;
    }
    public int getStalledRPMThreshold() {
        return stalledRPMThreshold;
    }
    public void setStalledRPMThreshold(int stalledRPMThreshold) {
        this.stalledRPMThreshold = stalledRPMThreshold;
    }
    public void enableStallDetection() {
        setStallDetection(true);
        Runnable mainRunnable = () -> {
            while (opModeIsActive()) {
            stalled = getStalled();
            if (getStallDetection()) {
                if (stalled) stallAction.run();
                else unStalledAction.run();
            }
            MasqUtils.sleep(100);
        }};
        Thread thread = new Thread(mainRunnable);
        thread.start();
    }

    public void setClosedLoop(boolean closedLoop) {
        this.closedLoop = closedLoop;
    }

    public double getPower () {
        return motorPower;
    }

    public MasqEncoder getEncoder () {
        return encoder;
    }

    public void setKp(double kp) {
        this.kp = kp;
    }
    public void setKi(double ki) {
        this.ki = ki;
    }
    public void setKd(double kd) {
        this.kd = kd;
    }

    private boolean opModeIsActive() {
        return MasqUtils.opModeIsActive();
    }
    public DcMotorController getController () {return motor.getController();}
    public int getPortNumber () {return motor.getPortNumber();}

    public void setMotorModel (MasqMotorModel model) {encoder.setModel(model);}

    public boolean isClosedLoop() {return closedLoop;}

    public double getMinPower() {return minPower;}

    public void setMinPower(double power) {minPower = power;}

    public void setWheelDiameter(double diameter) {encoder.setWheelDiameter(diameter);}

    public double getInches() {return encoder.getInches();}

    public double getTargetPower() {return targetPower;}

    public void reverseEncoder() {reversedEncoder = !reversedEncoder;}

    public void setDirection(DcMotor.Direction direction) {motor.setDirection(direction);}

    public String getName() {return nameMotor;}
    public String[] getDash() {
        return new String[] {
                "Current Position: " + getCurrentPosition(),
                "Velocity: " + getVelocity()};
    }
}