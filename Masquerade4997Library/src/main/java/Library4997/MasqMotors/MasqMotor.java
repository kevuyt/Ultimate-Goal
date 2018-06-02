package Library4997.MasqMotors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import Library4997.MasqSensors.MasqClock;
import Library4997.MasqSensors.MasqEncoder;
import Library4997.MasqSensors.MasqLimitSwitch;
import Library4997.MasqUtilities.Direction;
import Library4997.MasqUtilities.MasqHardware;
import Library4997.MasqUtilities.MasqUtils;

/**
 * This is a custom motor that includes stall detection and telemetry
 */
public class MasqMotor implements MasqHardware {
    private DcMotor motor;
    private boolean stallDetection = false;
    private String nameMotor;
    private int direction = 1;
    private double targetPower;
    private boolean velocityControlState = false;
    private double kp = 0.004, ki = 0, kd = 0;
    private double holdKp = 0.0002;
    private boolean closedLoop = true;
    public MasqEncoder encoder;
    private boolean holdPositionMode = false;
    private double targetPosition = 0;
    private double prevPos = 0;
    private double previousAcceleration = 0;
    private boolean stalled = false;
    private double previousVel = 0;
    private double previousVelTime = 0;
    private double previousTime = 0;
    private double destination = 0;
    public double currentPower;
    private double currentMax, currentMin;
    private double currentZero;
    private double holdItergral = 0;
    private double holdDerivitive = 0;
    private double previousJerk, prevAcceleration, previousAccelerationTime;
    private double holdPreviousError = 0;
    private double previousAccelerationSetTime;
    private double rpmIntegral = 0;
    private double rpmDerivative = 0;
    private double rpmPreviousError = 0;
    private int stalledRPMThreshold = 10;
    private boolean stateControl;
    private double currentPosition = 0, zeroEncoderPosition = 0, prevRate = 0;
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
    private boolean limitDetection, positionDetection, halfDetectionMin, halfDetectionMax;
    private MasqLimitSwitch minLim, maxLim = null;

    public MasqMotor(String name, HardwareMap hardwareMap){
        limitDetection = positionDetection = false;
        this.nameMotor = name;
        motor = hardwareMap.get(DcMotor.class, name);
        encoder = new MasqEncoder(this, MasqEncoder.MasqMotorModel.NEVEREST20);
    }
    public MasqMotor(String name, DcMotor.Direction direction, HardwareMap hardwareMap) {
        limitDetection = positionDetection = false;
        if (direction == DcMotor.Direction.REVERSE) this.direction = 1;
        this.nameMotor = name;
        motor = hardwareMap.dcMotor.get(name);
        motor.setDirection(direction);
        encoder = new MasqEncoder(this, MasqEncoder.MasqMotorModel.NEVEREST20);
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
        encoder.resetEncoder();
    }

    public void setClosedLoop(boolean closedLoop) {this.closedLoop = closedLoop;}
    public void runUsingEncoder() {motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);}
    public void setDistance (double distance) {
        resetEncoder();
        destination = distance * MasqUtils.CLICKS_PER_INCH;
    }

    public void runToPosition(Direction direction, double speed){
        MasqClock timeoutTimer = new MasqClock();
        resetEncoder();
        double clicksRemaining;
        double  power;
        do {
            clicksRemaining = (destination - Math.abs(motor.getCurrentPosition()));
            power = -direction.value * speed * ((clicksRemaining / destination) * 1.3);
            power = Range.clip(power, -1.0, +1.0);
            setVelocity(power);
        } while (opModeIsActive() && Math.abs(clicksRemaining) > 1 && !timeoutTimer.elapsedTime(1, MasqClock.Resolution.SECONDS));
    }
    boolean isBusy () {
        return motor.isBusy();
    }
    public void setBreakMode () {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double getCurrentPosition() {
        return encoder.getRelativePosition();
    }
    public double getAbsolutePosition () {
        return encoder.getAbsolutePosition();
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
    public void setVelocity(double power) {
        targetPower = power;
        double motorPower = calculateVelocityCorrection();
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
    private double calculateVelocityCorrection() {
        if (holdPositionMode) {
            double tChange = (System.nanoTime() - previousTime) / 1e9;
            double error = targetPosition - getCurrentPosition();
            holdItergral += error * tChange;
            holdDerivitive = (error - holdPreviousError) / tChange;
            targetPower = (direction * ((error * holdKp) +
                    (holdItergral * ki) + (holdDerivitive * kd)));
            holdPreviousError = error;
        }
        if (closedLoop) {
            double error, setRPM, currentRPM, motorPower;
            double tChange = System.nanoTime() - previousTime;
            tChange /= 1e9;
            setRPM = MasqUtils.NEVERREST_ORBITAL_20_RPM * targetPower;
            currentRPM = getVelocity();
            error = setRPM - currentRPM;
            rpmIntegral += error * tChange;
            rpmDerivative = (error - rpmPreviousError) / tChange;
            motorPower = (targetPower) + (direction * ((error * kp) +
                    (rpmIntegral * ki) + (rpmDerivative * kd)));
            rpmPreviousError = error;
            return motorPower;
        }
        else return targetPower;
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

    public double getKp() {return kp;}
    public void setKp(double kp) {this.kp = kp;}
    public double getKi() {return ki;}
    public void setKi(double ki) {this.ki = ki;}
    public double getKd() {return kd;}
    public void setKd(double kd) {this.kd = kd;}

    private boolean opModeIsActive() {
        return MasqUtils.opModeIsActive();
    }

    public void setMotorModel (MasqEncoder.MasqMotorModel model) {
        encoder.setModel(model);
    }

    public void setLazy() {
        holdPositionMode = false;
    }
    public void setStrong() {
        holdPositionMode = true;
        targetPosition = getCurrentPosition();
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

