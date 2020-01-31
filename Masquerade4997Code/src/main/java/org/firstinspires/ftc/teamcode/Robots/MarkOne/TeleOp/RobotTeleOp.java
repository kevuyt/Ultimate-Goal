package org.firstinspires.ftc.teamcode.Robots.MarkOne.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;

import Library4997.MasqResources.MasqUtils;
import Library4997.MasqWrappers.MasqLinearOpMode;

import static Library4997.MasqResources.MasqUtils.toggle;

/**
 * Created by Keval Kataria on 9/14/2019
 */
@TeleOp(name = "RobotTeleOp", group = "MarkOne")
public class RobotTeleOp extends MasqLinearOpMode {
    private MarkOne robot = new MarkOne();

    public static boolean currStateBlockGrabber=false, prevStateBlockGrabber =false, taskStateBlockGrabber =false;
    public static boolean currStateBlockRotator=false, prevStateBlockRotator =false, taskStateBlockRotator =false;
    public static boolean currStateCapper=false, prevStateCapper =false, taskStateCapper =false;

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initializeTeleop();

        while(!opModeIsActive()) {
            dash.create("Heading: ", robot.tracker.getHeading());
            dash.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            if (controller1.rightBumper() || controller1.leftBumper())
                robot.MECH(controller1,0.5, 0.2);
            else robot.MECH(controller1,1, 0.5);

            if (controller1.leftTriggerPressed()) robot.intake.setVelocity(-1);
            else if (controller1.rightTriggerPressed()) robot.intake.setVelocity(1);
            else robot.intake.setVelocity(0);

            robot.lift.setVelocity(0.75 * controller2.leftStickY());

            robot.tapeMeasure.setPower(controller2.rightStickY());

            toggle(controller2.x(),robot.blockGrabber);
            toggle(controller2.y(),robot.blockRotater);
            toggle(controller2.dPadUp(),robot.capper);

            robot.foundationHook.DriverControl(controller1);

            robot.sideGrabber.teleReset();

            dash.create("X: ",robot.tracker.getGlobalX());
            dash.create("Y: ",robot.tracker.getGlobalY());
            dash.create("Raw X: ",robot.tapeMeasure.getCurrentPosition());
            dash.create("Raw YL: ",robot.intake.motor2.getCurrentPosition());
            dash.create("Raw YR: ", robot.intake.motor1.getCurrentPosition());
            dash.create("XR stick: ", controller1.rightStickX());
            robot.tracker.updateSystem();
            dash.update();

            controller1.update();
            controller2.update();
        }
    }

    /*public void toggleBlockGrabber() {

        if (controller2.x()) {
            currStateBlockGrabber = true;
        } else {
            currStateBlockGrabber = false;
            if (prevStateBlockGrabber) {
                taskStateBlockGrabber = !taskStateBlockGrabber;
            }
        }

        prevStateBlockGrabber = currStateBlockGrabber;

        if (taskStateBlockGrabber) {
            robot.blockGrabber.setPosition(1);
        } else {
            robot.blockGrabber.setPosition(0);
        }
    }


    public void toggleBlockRotator() {

        if (controller2.y()) {
            currStateBlockRotator = true;
        } else {
            currStateBlockRotator = false;
            if (prevStateBlockRotator) {
                taskStateBlockRotator = !taskStateBlockRotator;
            }
        }

        prevStateBlockRotator = currStateBlockRotator;

        if (taskStateBlockRotator) {
            robot.blockRotater.setPosition(1);
        } else {
            robot.blockRotater.setPosition(0);
        }
    }

    public void toggleCapper() {

        if (controller2.dPadUp()) {
            currStateCapper = true;
        } else {
            currStateCapper = false;
            if (prevStateCapper) {
                taskStateCapper = !taskStateCapper;
            }
        }

        prevStateCapper = currStateCapper;

        if (taskStateCapper) {
            robot.capper.setPosition(1);
        } else {
            robot.capper.setPosition(0);
        }
    }*/
}