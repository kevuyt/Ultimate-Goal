package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqMotors.MasqMotorSystem;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqSensors.MasqPositionTracker;
import Library4997.MasqSensors.MasqTouchSensor;
import Library4997.MasqUtilities.Direction;
import Library4997.MasqUtilities.MasqEncoderModel;
import Library4997.MasqUtilities.MasqUtils;
import SubSystems4997.MasqRobot;

/**
 * Created by Archishmaan Peyyety on 6/2/18.
 * Project: MasqLib
 */

public class Robot extends MasqRobot {
    private HardwareMap hardwareMap;
    public MasqTouchSensor touchSensor;
    public MasqPositionTracker positionTracker;
    public MasqMotorSystem yTranslator;
    public MasqMotor leftMotor, rightMotor;
    private MasqClock timeoutClock = new MasqClock();
    @Override
    public void mapHardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        yTranslator = new MasqMotorSystem("y1", "y2", "y", this.hardwareMap);
        touchSensor = new MasqTouchSensor("touchSensor", this.hardwareMap);
        leftMotor = new MasqMotor("leftMotor", DcMotor.Direction.REVERSE, this.hardwareMap);
        rightMotor = new MasqMotor("rightMotor", DcMotor.Direction.FORWARD, this.hardwareMap);
        positionTracker = new MasqPositionTracker(this.hardwareMap, leftMotor, MasqEncoderModel.USDIGITAL_E4T,
                rightMotor, MasqEncoderModel.NEVEREST20);
    }

    public void robotDrive (Direction direction, int distance, double speed) {
        double error = distance - leftMotor.getAbsolutePosition();
        while (error <= 0.1 && opModeIsActive()) {
            double motorPower = speed * (error/distance);
            leftMotor.setPower(motorPower);
            rightMotor.setPower(motorPower);
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void drive(double distance, double speed) {
        leftMotor.resetEncoder();
        double targetClicks = (int)(distance * MasqUtils.CLICKS_PER_INCH);
        double error;
        double power;
        do {
            error = (int) (targetClicks - Math.abs(leftMotor.getCurrentPosition()));
            power = ((error / targetClicks)) * speed;
            power = Range.clip(power, -1.0, +1.0);
            leftMotor.setPower(power);
            rightMotor.setPower(power);
        } while (opModeIsActive() && (error / targetClicks) > 0.05);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}
