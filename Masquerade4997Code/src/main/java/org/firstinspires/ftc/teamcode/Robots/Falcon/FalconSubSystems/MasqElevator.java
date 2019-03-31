package org.firstinspires.ftc.teamcode.Robots.Falcon.FalconSubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqSubSystem;
import Library4997.MasqWrappers.DashBoard;
import Library4997.MasqWrappers.MasqController;

/**
 * Created by Archishmaan Peyyety on 3/30/19.
 * Project: MasqLib
 */
public class MasqElevator implements MasqSubSystem {
    private MasqMotor lift;
    private static double rotatorMotion;
    private MasqPIDController pidController = new MasqPIDController(0.005, 0, 0);
    private double targetPosition;
    public MasqElevator (HardwareMap hardwareMap) {
        lift = new MasqMotor("lift", MasqMotorModel.ORBITAL20, DcMotor.Direction.REVERSE, hardwareMap);
        lift.resetEncoder();
    }

    @Override
    public void DriverControl(MasqController controller) {
        if (controller.rightBumper()) {
            targetPosition = lift.getCurrentPosition();
            lift.setPower(-1);
        }
        else if (controller.rightTriggerPressed()) {
            targetPosition = lift.getCurrentPosition();
            lift.setPower(1);
        }
        else if (rotatorMotion == 0) targetPosition = -1100;
        else if (rotatorMotion == 1) targetPosition = -1200;
        else if (rotatorMotion == 2 || (!controller.rightTriggerPressed() && !controller.rightBumper())) lift.setPower(pidController.getOutput(lift.getCurrentPosition(), targetPosition));
        DashBoard.getDash().create("lift: ", lift.getCurrentPosition());
    }

    public static void setRotatorMotion(double rotatorMotion1) {
        rotatorMotion = rotatorMotion1;
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
