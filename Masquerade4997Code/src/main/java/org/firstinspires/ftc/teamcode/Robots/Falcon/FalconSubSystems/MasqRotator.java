package org.firstinspires.ftc.teamcode.Robots.Falcon.FalconSubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqRobot;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqSubSystem;
import Library4997.MasqWrappers.MasqController;

/**
 * Created by Archishmaan Peyyety on 10/17/18.
 * Project: MasqLib
 */

public class MasqRotator implements MasqSubSystem {
    public MasqMotor rotator;
    private double targetPosition;
    private double basePower = 0.9;
    private double kp_kp = 5;
    private double baseDownPower = 0.9;
    private double rotatorPower = basePower;
    private double downPower = -0.1;
    private double liftPosition = 0;
    private double kp = 0.01, ki, kd;
    public MasqPIDController output = new MasqPIDController(0.01, 0.0, 0.00);
    public MasqRotator (HardwareMap hardwareMap) {
        rotator = new MasqMotor("rotator", MasqMotorModel.NEVEREST40, DcMotor.Direction.FORWARD, hardwareMap);
        rotator.setClosedLoop(true);
        rotator.resetEncoder();
    }
    @Override
    public void DriverControl(MasqController controller) {
        kp = (1e-6 * -liftPosition) + 0.001;
        ki = 0.0;
        kd = 0.0;
        rotatorPower = (0.0001 * kp_kp * -liftPosition) + basePower;
        downPower = (0.001 * kp_kp * Math.abs(rotator.getCurrentPosition())) + baseDownPower;
        if (controller.leftTriggerPressed()) rotator.setPower(rotatorPower);
        else if (controller.rightTriggerPressed()) rotator.setPower(-downPower);

        if (controller.rightTrigger() > 0 || controller.leftTrigger() > 0) targetPosition = rotator.getCurrentPosition();
        else {
            double currentPosition = rotator.getCurrentPosition();
            rotator.setPower(output.getOutput(currentPosition, targetPosition));
        }
        output.setKp(kp);
        output.setKi(ki);
        output.setKd(kd);
    }
    public void setLiftPosition (double liftPosition) {
        this.liftPosition = liftPosition;
    }

    public void setAngle (double angle, Direction direction) {

    }

    public void setPower (Direction direction, double time) {
        MasqClock clock = new MasqClock();
        while (MasqRobot.opModeIsActive() && !clock.elapsedTime(time, MasqClock.Resolution.SECONDS))
            rotator.setPower(1 * direction.value);
    }

    public double getPosition() {
        return rotator.getCurrentPosition();
    }

    public double getRawPower () {
        return rotator.getPower();
    }

    public double getAngle() {
        double angle =  (rotator.getCurrentPosition() * rotator.getEncoder().getClicksPerRotation()) / 360;
        return angle / 170;
    }

    @Override
    public String getName() {
        return null;
    }

    @Override
    public MasqHardware[] getComponents() {
        return new MasqHardware[0];
    }
}
