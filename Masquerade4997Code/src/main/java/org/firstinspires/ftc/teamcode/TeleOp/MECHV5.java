package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqUtilities.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 2/7/18.
 */
@TeleOp(name = "MECHV5", group = "Autonomus")
public class MECHV5 extends MasqLinearOpMode implements Constants {
    double currentRelicPower = LIFT_UP, position = 0;
    boolean disabled = false;
    boolean intakeState = false;
    Direction direction = Direction.FORWARD;
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.initializeTeleop();
        robot.relicLift.setClosedLoop(false);
        robot.intake.motor1.setStalledAction(new Runnable() {
            @Override
            public void run() {
                robot.intake.setPower(OUTAKE);
            }
        });
        robot.intake.motor1.setUnStalledAction(new Runnable() {
            @Override
            public void run() {
                robot.intake.setPower(INTAKE);
            }
        });
        robot.yWheel.resetEncoder();
        while (!opModeIsActive()) {
            dash.create("YWheel: ", robot.yWheel.getInches());
            dash.create("Inverted: ", -robot.yWheel.getPosition());
            dash.update();
        }
        waitForStart();
        robot.relicAdjuster.setPosition(0);
        robot.intake.motor1.enableStallDetection();
        while (opModeIsActive()) {
            if (controller1.y()) direction = Direction.FORWARD;
            if (controller1.b()) direction = Direction.BACKWARD;
            if (controller2.leftStickY() < -0.5) position = 1;
            else if (controller2.leftStickY() > 0.5) position = 0;
            else if (controller2.leftStickX() > 0.5) position = 0.5;
            else if (controller2.leftStickX() < -0.5) position = 0.5;
            robot.relicAdjuster.setPosition(position);
            if (controller1.rightBumper()) {
                robot.intake.motor1.setStallDetection(false);
                robot.intake.setPower(0);
                intakeState = false;
            }
            else if (controller1.rightTriggerPressed()) {
                robot.intake.setPower(INTAKE);
                robot.intake.motor1.setStallDetection(true);
                intakeState = true;
            }
            if (controller1.x()) robot.jewelArmRed.setPosition(JEWEL_RED_IN);
            if (controller2.b()) robot.relicGripper.setPosition(CLAW_OPENED);
            else if (controller2.y()) robot.relicGripper.setPosition(CLAW_CLOSED);
            if (controller2.rightBumper()) robot.lift.setPower(LIFT_UP);
            else if (controller2.rightTriggerPressed()) robot.lift.setPower(LIFT_DOWN);
            else robot.lift.setPower(0);
            if (controller2.leftBumper()) robot.relicLift.setPower(currentRelicPower);
            else if (controller2.leftTriggerPressed()) {robot.relicLift.setPower(-controller2.leftTrigger());}
            else robot.relicLift.setPower(0);
            robot.flipper.DriverControl(controller2);
            robot.MECH(controller1, direction, disabled);
            disabled = controller1.a();
            controller1.update();
            controller2.update();
        }
    }
}