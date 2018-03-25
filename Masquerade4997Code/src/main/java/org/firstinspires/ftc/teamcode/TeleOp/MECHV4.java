package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqUtilities.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 2/7/18.
 */
@TeleOp(name = "MECHV4", group = "Autonomus")
public class MECHV4 extends MasqLinearOpMode implements Constants {
    double currentRelicPower = LIFT_UP;
    boolean backWord = false;
    double position = 0;
    double pitch = 0;
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.initializeTeleop();
        robot.relicLift.setClosedLoop(false);
        while (!opModeIsActive()) {
            dash.create(robot.imu);
            dash.update();
        }
        waitForStart();
        robot.relicAdjuster.setPosition(0);
        while (opModeIsActive()) {
            if (controller1.leftBumperOnPress() && !backWord) {
                robot.MECH(controller1, Direction.FORWARD);
                backWord = true;
            }
            else if (controller1.leftBumperOnPress() && backWord) {
                robot.MECH(controller1, Direction.FORWARD);
                backWord = false;
            }
            if (controller1.rightBumper()) robot.intake.setPower(INTAKE);
            else if (controller1.rightTriggerPressed()) robot.intake.setPower(OUTAKE);
            else  robot.intake.setPower(0);
            if (controller2.leftStickY() < -0.5) position = 1;
            else if (controller2.leftStickY() > 0.5) position = 0;
            else if (controller2.leftStickX() > 0.5) position = 0.5;
            else if (controller2.leftStickX() < -0.5) position = 0.5;
            if (controller1.x()) {robot.jewelArmRed.setPosition(JEWEL_RED_IN);}
            if (controller1.a()) pitch = robot.imu.getPitch();
            if (controller2.b()) robot.relicGripper.setPosition(CLAW_OPENED);
            else if (controller2.y()) robot.relicGripper.setPosition(CLAW_CLOSED);
            if (controller2.rightBumper()) robot.lift.setPower(LIFT_UP);
            else if (controller2.rightTriggerPressed()) robot.lift.setPower(LIFT_DOWN);
            else robot.lift.setPower(0);
            if (controller2.leftBumper()) robot.relicLift.setPower(currentRelicPower);
            else if (controller2.leftTriggerPressed()) {robot.relicLift.setPower(-controller2.leftTrigger());}
            else robot.relicLift.setPower(0);
            robot.relicAdjuster.setPosition(position);
            robot.flipper.DriverControl(controller2);
            controller1.update();
            controller2.update();
            robot.sleep(70);
        }
    }
    public void doBalance() {
        if (controller1.rightTriggerPressed()) {
            robot.driveTrain.setPower(robot.imu.getPitch() / pitch);
        }
        else {
            robot.driveTrain.setPower(0);
        }
    }


}