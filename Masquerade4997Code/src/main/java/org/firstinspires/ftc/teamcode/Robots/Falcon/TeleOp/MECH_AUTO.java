package org.firstinspires.ftc.teamcode.Robots.Falcon.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;

import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
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
    private double maxSpeed = 0.7;
    private MasqPIDController speedController = new MasqPIDController(0.03, 0.0, 0.00001);
    private double scoreExtension = -2800;
    private double collectionExtension = -3500;
    private double scoreRotation = -1302;
    private double midRotation = -600;
    private double collectRotation = 300;
    private double targetHeading = 0;
    private MasqVector target = new MasqVector(0, 0);
    private MasqVector current = new MasqVector(0, 0);
    private double prevRotatorPosition = 0;
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
            if (falcon.rotator.rotator.motor2.getCurrentPosition() > midRotation
                    && prevRotatorPosition < midRotation) falcon.lift.setTargetPosition(collectionExtension);
            else if (falcon.rotator.rotator.motor2.getCurrentPosition() < midRotation
                    && prevRotatorPosition > midRotation) falcon.lift.setTargetPosition(scoreExtension);

            if (controller1.leftTriggerPressed()) falcon.collector.setPower(-.5);
            else if (controller1.leftBumper() || controller2.b()) falcon.collector.setPower(.5);
            else falcon.collector.setPower(0);

            if (controller2.b()) falcon.dumper.setPosition(DUMPER_OUT);
            else falcon.dumper.setPosition(DUMPER_IN);

            if (controller2.leftStickY() < 0 && !falcon.hangTopSwitch.isPressed()) falcon.hang.setPower(-1);
            else if (controller2.leftStickY() > 0 && !falcon.hangBottomSwitch.isPressed()) falcon.hang.setPower(1);
            else falcon.hang.setBreakMode();

            if (controller2.x()) {
                scoreRotation = falcon.rotator.rotator.motor2.getCurrentPosition();
                scoreExtension = falcon.lift.lift.getCurrentPosition();
            }

            if (controller1.x()) {
                target = new MasqVector(falcon.tracker.getGlobalX(), falcon.tracker.getGlobalY());
                targetHeading = falcon.tracker.getHeading();
            }

            if (controller1.y()) {
                /*if (!falcon.rotateTopSwitch.isPressed() &&
                        falcon.rotator.rotator.motor2.getCurrentPosition() > scoreRotation)
                    falcon.rotator.rotator.setPower(-1);*/
                falcon.rotator.setHoldPosition(scoreRotation);
                updateDrivetrainToPath();
            }
            else falcon.MECH(controller1);
            updateSystem();
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
    public void updateSystem() {
        prevRotatorPosition = falcon.rotator.rotator.motor2.getCurrentPosition();
        current = new MasqVector(falcon.tracker.getGlobalX(), falcon.tracker.getGlobalY());
        falcon.rotator.DriverControl(controller2);
        falcon.lift.DriverControl(controller1);
        falcon.tracker.updateSystem();
        dash.create("TX: ", target.getX());
        dash.create("TY: ", target.getY());
        dash.create("X: ", falcon.tracker.getGlobalX());
        dash.create("Y: ", falcon.tracker.getGlobalY());
        dash.create("H: ", falcon.tracker.getHeading());
        dash.create("Rot Lim: ", falcon.rotateTopSwitch.isPressed());
        dash.update();
    }
    public void updateDrivetrainToPath() {
        MasqVector displacement = current.displacement(target);
        double speed = speedController.getOutput(displacement.getMagnitude());
        if (Math.abs(speed) > maxSpeed) speed = maxSpeed * (speed / Math.abs(speed));
        double pathAngle = 90 - Math.toDegrees(Math.atan2(displacement.getY(), displacement.getX()));
        falcon.driveTrain.setPowerMECH(pathAngle + falcon.tracker.getHeading(), speed, targetHeading);
        dash.create("Speed: ", speed);
    }
}
