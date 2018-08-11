package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.Creed;

import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;


/**
 * Created by Archish on 2/7/18.
 */
@TeleOp(name = "MECH", group = "Autonomus")
public class MECH extends MasqLinearOpMode implements Constants {
    private Creed creed = new Creed();
    Direction direction = Direction.FORWARD;
    private double position = 1;
    private double currentRelicPower = LIFT_UP;
    public void runLinearOpMode() throws InterruptedException {
        creed.blueLineDetector.setPassive();
        creed.redLineDetector.setPassive();
        creed.jewelColorRed.setPassive();
        creed.singleBlock.setPassive();
        creed.doubleBlock.setPassive();
        creed.initializeTeleop();
        creed.relicLift.setClosedLoop(false);
        creed.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create(creed.driveTrain.leftDrive.motor1.getEncoder().getModel().CPR());
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            if (controller1.y()) direction = Direction.FORWARD;
            if (controller1.b()) direction = Direction.BACKWARD;
            if (controller1.rightBumper()) creed.intake.setPower(INTAKE);
            else if (controller1.rightTriggerPressed()) creed.intake.setPower(OUTAKE);
            else  creed.intake.setPower(0);
            if (controller2.leftStickY() < -0.5) position = 0;
            else if (controller2.leftStickY() > 0.5) position = .98;
            else if (controller2.leftStickX() > 0.5) position = 0.5;
            else if (controller2.leftStickX() < -0.5) position = 0.5;
            creed.relicAdjuster.setPosition(position);
            if (controller1.x()) creed.jewelArmRed.setPosition(JEWEL_RED_IN);
            if (controller2.b()) creed.relicGripper.setPosition(CLAW_OPENED);
            else if (controller2.y()) creed.relicGripper.setPosition(CLAW_CLOSED);
            if (controller2.rightBumper()) creed.lift.setPower(LIFT_UP);
            else if (controller2.rightTriggerPressed()) creed.lift.setPower(LIFT_DOWN);
            else creed.lift.setPower(0);
            if (controller2.leftBumper()) creed.relicLift.setPower(currentRelicPower);
            else if (controller2.leftTriggerPressed()) {creed.relicLift.setPower(-controller2.leftTrigger());}
            else creed.relicLift.setPower(0);
            creed.flipper.DriverControl(controller2);
            creed.MECH(controller1, direction, false);
        }
    }
}