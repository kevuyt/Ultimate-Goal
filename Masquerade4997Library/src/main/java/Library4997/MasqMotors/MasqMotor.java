package Library4997.MasqMotors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqResources.MasqUtils;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqSensors.MasqEncoder;
import Library4997.MasqSensors.MasqLimitSwitch;

/**
 * This is a custom motor that includes stall detection and telemetry
 */
public class MasqMotor implements MasqHardware {
    private double minPower = 0;
    private DcMotor motor;
    private boolean stallDetection = false;
    private String nameMotor;
    private int direction = 1;
    private double targetPower;
    private boolean velocityControlState = false;
    private double kp = 0.1, ki = 0, kd = 0;
    private MasqEncoder encoder;
    private double targetPosition = 0;
    private double prevPos = 0;
    private double previousAcceleration = 0;
    private boolean stalled = false;
    private double previousVel = 0;
    private double previousVelTime = 0;
    private double previousTime = 0;
    private double destination = 0;
    private double motorPower;
    private double currentMax, currentMin;
    private double currentZero;
    private double previousJerk, prevAcceleration, previousAccelerationTime;
    private double previousAccelerationSetTime;
    private double rpmIntegral = 0;
    private double rpmDerivative = 0;
    private double rpmPreviousError = 0;
    private int stalledRPMThreshold = 10;
    private boolean stateControl;
    private double prevRate = 0;
    private Runnable stallAction = new Runnable() {
        @Override
        public void run() {

        }
    },
    unStalledAction = new Runnable() {
        @Override
        public void run() {

        }
    };

    private double minPosition, maxPosition;
    private boolean
            limitDetection = false,
            positionDetection = false,
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
        if (direction == DcMotor.Direction.REVERSE) this.direction = 1;
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

    public void runWithoutEncoders () {motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);}
    public void resetEncoder() {
        encoder.resetEncoder();
    }

    public void runUsingEncoder() {motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);}
    public void setDistance (double distance) {
        resetEncoder();
        destination = distance;
    }
    public void runToPosition(int inches, double speed){
        MasqClock clock = new MasqClock();
        resetEncoder();
        double clicks = -inches * encoder.getClicksPerInch();
        motor.setTargetPosition((int) clicks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setVelocity(speed);
        while (opModeIsActive() && motor.isBusy() &&
                !clock.elapsedTime(5, MasqClock.Resolution.SECONDS)) {}
        setVelocity(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    boolean isBusy () {
        return motor.isBusy();
    }
    public void setBreakMode () {
        motor.setPower(0);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void unBreakMode () {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public double getCurrentPosition() {
        return encoder.getRelativePosition();
    }
    public double getAbsolutePosition () {
        return motor.getCurrentPosition();
    }
    public double getVelocity() {
        double deltaPosition = getCurrentPosition() - prevPos;
        double tChange = System.nanoTime() - previousTime;
        previousTime = System.nanoTime();
        tChange = tChange / 1e9;
        prevPos = getCurrentPosition();
        double rate = deltaPosition / tChange;
        rate = (rate * 60) / encoder.getClicksPerRotation();
        if (rate != 0) return rate;
        else {
            prevRate = rate;
            return prevRate;
        }
    }
    public double getAngle () {
        return (motor.getCurrentPosition() * encoder.getClicksPerRotation()) / 360;
    }
    public void setPower (double power) {
        motor.setPower(power);
    }
    public void setVelocity(double power) {
        targetPower = power;
        motorPower = calculateVelocityCorrection();
        if (!closedLoop) motorPower = targetPower;
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
    private double calculateVelocityCorrection() {
        double error, setRPM, currentRPM, tChange;
        tChange = System.nanoTime() - previousTime;
        tChange /= 1e9;
        setRPM = encoder.getRPM() * targetPower;
        currentRPM = getVelocity();
        error = setRPM - currentRPM;
        rpmIntegral += error * tChange;
        rpmDerivative = (error - rpmPreviousError) / tChange;
        double power = (targetPower) + (direction * ((error * kp) +
                (rpmIntegral * ki) + (rpmDerivative * kd)));
        rpmPreviousError = error;
        previousTime = System.nanoTime();
        return power;
    }
    public double getAcceleration () {
        double deltaVelocity = getVelocity() - previousVel;
        double tChange = System.nanoTime() - previousVelTime;
        tChange = tChange / 1e9;
        previousVel = getVelocity();
        double acceleration = deltaVelocity / tChange;
        previousVelTime = System.nanoTime();
        if (acceleration != 0) return acceleration;
        else {
            previousAcceleration = acceleration;
            return previousAcceleration;
        }
    }
    public void setAcceleration (double accelerationRPMM) {
        double tChange = System.nanoTime() - previousAccelerationSetTime;
        double currentRPM = getVelocity();
        double newRPM = currentRPM + (accelerationRPMM * tChange);
        setVelocity(calculateAccelerationCorrection(newRPM));
        previousAccelerationSetTime = System.nanoTime();
    }
    public double calculateAccelerationCorrection (double targetAcceleration) {
        double error, currentRPM, motorPower;
        double tChange = System.nanoTime() - previousTime;
        tChange /= 1e9;
        currentRPM = getAcceleration();
        error = targetAcceleration - currentRPM;
        rpmIntegral += error * tChange;
        rpmDerivative = (error - rpmPreviousError) / tChange;
        motorPower = (targetPower) + (direction * ((error * kp) +
                (rpmIntegral * ki) + (rpmDerivative * kd)));
        rpmPreviousError = error;
        return motorPower;
    }
    public double getJerk () {
        double deltaAcceleration = getAcceleration() - prevAcceleration;
        double tChange = System.nanoTime() - previousAccelerationTime;
        tChange = tChange / 1e9;
        prevAcceleration = getVelocity();
        double jerk = deltaAcceleration / tChange;
        previousAccelerationTime = System.nanoTime();
        if (jerk != 0) return jerk;
        else {
            previousJerk = jerk;
            return jerk;
        }
    }
    public void setVelocityControlState(boolean velocityControlState) {
        this.velocityControlState = velocityControlState;
    }
    public void startVelocityControl () {
        setVelocityControlState(true);
        Runnable velocityControl = new Runnable() {
            @Override
            public void run() {
                while (opModeIsActive() && velocityControlState) {
                    setVelocity(targetPower);
                }
            }
        };
        Thread velocityThread = new Thread(velocityControl);
        velocityThread.start();
    }
    public void setControlStateUpdate (boolean velocityControlState) {this.stateControl = velocityControlState; }
    public void startStateUpdate () {
        setControlStateUpdate(true);
        new Thread(new Runnable() {
            @Override
            public void run() {
                while (opModeIsActive()) {
                    if (stateControl) getJerk();
                    MasqUtils.sleep(10);
                }
            }
        }).start();
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
        Runnable mainRunnable = new Runnable() {
            @Override
            public void run() {
                while (opModeIsActive()) {
                    stalled = getStalled();
                    if (getStallDetection()) {
                        if (stalled) stallAction.run();
                        else unStalledAction.run();
                    }
                    MasqUtils.sleep(100);
                }
            }
        };
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

    public double getKp() {return kp;}
    public void setKp(double kp) {
        this.kp = kp;
    }
    public double getKi() {return ki;}
    public void setKi(double ki) {
        this.ki = ki;
    }
    public double getKd() {return kd;}
    public void setKd(double kd) {
        this.kd = kd;
    }

    private boolean opModeIsActive() {
        return MasqUtils.opModeIsActive();
    }
    public DcMotorController getController () {
        return motor.getController();
    }
    public int getPortNumber () {
        return motor.getPortNumber();
    }

    public void setMotorModel (MasqMotorModel model) {
        encoder.setModel(model);
    }

    public boolean isClosedLoop() {
        return closedLoop;
    }

    public double getMinPower() {
        return minPower;
    }

    public void setMinPower(double minPower) {
        this.minPower = minPower;
    }

    public String getName() {
        return nameMotor;
    }
    public String[] getDash() {
        return new String[] {
                "Current Position: " + Double.toString(getCurrentPosition()),
                "Velocity: " + Double.toString(getVelocity()),
                "Acceleration: " + Double.toString(getAcceleration())};
    }
}

