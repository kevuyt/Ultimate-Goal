package org.firstinspires.ftc.teamcode.Robots.Falcon.FalconSubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqRobot;
import Library4997.MasqSubSystem;
import Library4997.MasqWrappers.DashBoard;
import Library4997.MasqWrappers.MasqController;

/**
 * Created by Archishmaan Peyyety on 11/7/18.
 * Project: MasqLib
 */

public class MasqElevator implements MasqSubSystem {
    private MasqMotor lift;
    private double targetPosition;
    private double kp;

    private MasqPIDController output = new MasqPIDController(0.005, 0, 0.001);
    public MasqElevator (HardwareMap hardwareMap) {
        lift = new MasqMotor("lift", MasqMotorModel.NEVEREST60, DcMotor.Direction.FORWARD, hardwareMap);
        lift.resetEncoder();
    }

    @Override
    public void DriverControl(MasqController controller) {
        kp = (5e-7 * -lift.getCurrentPosition()) + 0.001;
        if (controller.rightBumper()) lift.setPower(-1);
        else if (controller.rightTriggerPressed()) lift.setPower(1);

        if (controller.rightBumper() || controller.rightTriggerPressed()) targetPosition = lift.getCurrentPosition();
        else {
            double currentPosition = lift.getCurrentPosition();
            lift.setPower(output.getOutput(currentPosition, targetPosition));
        }
        output.setKp(kp);
    }
    public void runToPosition (Direction direction, double position) {
        double ticksRemaining = Math.abs(position - Math.abs(lift.getCurrentPosition()));
        while (ticksRemaining > 200 && MasqRobot.opModeIsActive()) {
            ticksRemaining = Math.abs(position - Math.abs(lift.getCurrentPosition()));
            lift.setPower(direction.value);
            DashBoard.getDash().create(ticksRemaining);
            DashBoard.getDash().update();
        }
        lift.setPower(0);
    }

    public double getCurrentPosition () {
        return lift.getCurrentPosition();
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
