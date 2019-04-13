package org.firstinspires.ftc.teamcode.Robots.Falcon.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;

import Library4997.MasqResources.MasqMath.MasqVector;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 10/27/18.
 * Project: MasqLib
 */
@TeleOp(name = "MECH_AUTO", group = "NFS")
public class MECH_AUTO extends MasqLinearOpMode implements Constants {
    private Falcon falcon = new Falcon();
    private String status = "";
    double scoreExtension = -2000;
    double collectionExtension = -1500;
    private double targetHeading = 0;
    private double targetInches = 0;
    private MasqVector target = new MasqVector(0, 0);
    private MasqVector current = new MasqVector(0, 0);
    MasqVector inital = new MasqVector(0, 0);
    MasqVector pathDisplacment = inital.displacement(target);
    private boolean prevA = false;
    @Override
    public void runLinearOpMode()  {
        robotInit();
        current = new MasqVector(falcon.tracker.getGlobalX(), falcon.tracker.getGlobalY());
        while (!opModeIsActive()) {
            falcon.tracker.updateSystem();
            dash.create("X: ", falcon.tracker.getGlobalX());
            dash.create("Y: ", falcon.tracker.getGlobalY());
            dash.create("H: ", falcon.tracker.getHeading());
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            /*if (falcon.rotator.getAngle() < 45) falcon.lift.setTargetPosition(collectionExtension);
            else falcon.lift.setTargetPosition(scoreExtension);

            if (controller1.leftTriggerPressed()) falcon.collector.setPower(-.5);
            else if (controller1.leftBumper() || controller2.b()) falcon.collector.setPower(.5);
            else falcon.collector.setPower(0);

            if (controller2.b()) falcon.dumper.setPosition(DUMPER_OUT);
            else falcon.dumper.setPosition(DUMPER_IN);

            if (controller2.leftStickY() < 0 && !falcon.topLimit.isPressed()) falcon.hang.setPower(-1);
            else if (controller2.leftStickY() > 0 && !falcon.downLimit.isPressed()) falcon.hang.setPower(1);
            else falcon.hang.setBreakMode();*/

            if (controller1.a() && !prevA) {
                initializePath();
                status = "path init";
            }
            else if (controller1.a() && prevA) {
                updateDrivetrainToPath();
                status = "updating dt";
            }

            else {
                falcon.MECH(controller1);
                status = "mech";
            }
            prevA = controller1.a();
            current = new MasqVector(falcon.tracker.getGlobalX(), falcon.tracker.getGlobalY());
            /*falcon.rotator.DriverControl(controller2);
            falcon.lift.DriverControl(controller1);*/
            falcon.tracker.updateSystem();
            dash.create("Status: ", status);
            dash.create("A: ", controller1.a());
            dash.create("X: ", falcon.tracker.getGlobalX());
            dash.create("Y: ", falcon.tracker.getGlobalY());
            dash.create("H: ", falcon.tracker.getHeading());
            dash.update();

        }
    }
    public void robotInit() {
        falcon.setStartOpenCV(false);
        falcon.mapHardware(hardwareMap);
        falcon.initializeTeleop();
        falcon.hang.setClosedLoop(true);
        falcon.driveTrain.setClosedLoop(true);
        falcon.dumper.setPosition(DUMPER_IN);
    }
    public void initializePath() {
       targetInches = current.distanceToVector(target);
    }
    public void updateDrivetrainToPath() {
        MasqVector displacement = current.displacement(target);
        double speed = (displacement.getMagnitude() / targetInches) * 0.5;
        double pathAngle = 90 - Math.toDegrees(Math.atan2(displacement.getY(), displacement.getX()));
        falcon.driveTrain.setPowerMECH(pathAngle + falcon.tracker.getHeading(), speed, targetHeading);
        dash.create("Speed: ", speed);
        dash.create("Target: ", targetHeading);
        dash.create("path: ", pathAngle);
    }
}
