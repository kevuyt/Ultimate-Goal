package Library4997.MasqMotors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqHardware;
import Library4997.MasqRobot;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqExternal.Direction;
import Library4997.MasqExternal.PID_CONSTANTS;
import Library4997.MasqSensors.MasqLimitSwitch;

/**
 * This is a custom motor that includes stall detection and telemetry
 */
public class MasqMotor implements PID_CONSTANTS, MasqHardware {
    private DcMotor motor;
    private String nameMotor;
    private double prevPos= 0;
    private double previousTime = 0;
    private double destination = 0;
    private double currentPosition = 0, zeroEncoderPosition = 0 , prevRate = 0;
    private MasqLimitSwitch minLim, maxLim = null;
    private boolean limitDetection;
    public MasqMotor(String name, HardwareMap hardwareMap){
        limitDetection = false;
        this.nameMotor = name;
        motor = hardwareMap.get(DcMotor.class, name);
    }
    public MasqMotor(String name, DcMotor.Direction direction, HardwareMap hardwareMap) {
        limitDetection = false;
        this.nameMotor = name;
        motor = hardwareMap.dcMotor.get(name);
        motor.setDirection(direction);
    }
    public MasqMotor setLimits(MasqLimitSwitch min, MasqLimitSwitch max){
        maxLim = max; minLim = min;
        limitDetection = true;
        return this;
    }
    public void runWithoutEncoders () {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void resetEncoder() {
        zeroEncoderPosition = motor.getCurrentPosition();
        currentPosition = 0;
    }
    public void setPower (double power) {
        if (!limitDetection)
            motor.setPower(power);
        else {
            if (maxLim.isPressed() && minLim.isPressed())
                motor.setPower(power);
            else
                motor.setPower(0);
        }
    }
    public void runUsingEncoder() {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setDistance (double distance) {
        resetEncoder();
        destination = distance;
    }
    private boolean opModeIsActive() {
        return MasqRobot.getInstance(null).opModeIsActive();
    }
    public void runToPosition(Direction direction, double speed){
        resetEncoder();
        int targetClicks = (int)(destination * CLICKS_PER_CM);
        int clicksRemaining;
        double inchesRemaining;
        double power;
        do {
            clicksRemaining = (int) (targetClicks - Math.abs(getCurrentPos()));
            inchesRemaining = clicksRemaining / CLICKS_PER_CM;
            power = direction.value * speed * inchesRemaining * KP_STRAIGHT;
            setPower(power);
        } while (opModeIsActive() && inchesRemaining > 0.5);
        setPower(0);
    }
    boolean isBusy () {
        return motor.isBusy();
    }
    public void setBreakMode () {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public double getCurrentPos () {
        currentPosition = motor.getCurrentPosition() - zeroEncoderPosition;
        return currentPosition;
    }
    public double getPower() {
        return motor.getPower();
    }
    public double getRate () {
        double posC = getCurrentPos() - prevPos;
        double tChange = System.nanoTime() - previousTime;
        previousTime = System.nanoTime();
        tChange = tChange / 1e9;
        prevPos = getCurrentPos();
        double rate = posC / tChange;
        if (rate != 0)
            return rate;
        else {
            prevRate = rate;
            return prevRate;
        }
    }
    public void sleep (int sleepTime) {
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    public String getName() {
        return nameMotor;
    }

    public String[] getDash() {
        return new String[] {"Current Position" + Double.toString(getCurrentPos())};
    }
}

