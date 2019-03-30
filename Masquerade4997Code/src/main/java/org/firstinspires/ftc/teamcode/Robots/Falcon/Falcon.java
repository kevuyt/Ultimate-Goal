package org.firstinspires.ftc.teamcode.Robots.Falcon;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.DogeForia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Robots.Falcon.FalconSubSystems.MasqElevator;
import org.firstinspires.ftc.teamcode.Robots.Falcon.FalconSubSystems.MasqRotator;
import org.firstinspires.ftc.teamcode.Robots.Falcon.Resources.BlockPlacement;

import Library4997.MasqControlSystems.MasqPID.MasqPIDPackage;
import Library4997.MasqControlSystems.MasqPurePursuit.MasqPositionTracker;
import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqResources.MasqUtils;
import Library4997.MasqRobot;
import Library4997.MasqSensors.MasqAdafruitIMU;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqSensors.MasqLimitSwitch;
import Library4997.MasqSensors.MasqVoltageSensor;
import Library4997.MasqServos.MasqCRServoSystem;
import Library4997.MasqServos.MasqServo;
import Library4997.MasqWrappers.DashBoard;

/**
 * Created by Archishmaan Peyyety on 6/2/18.
 * Project: MasqLib
 */

public class Falcon extends MasqRobot {
    public MasqAdafruitIMU imu;
    public MasqLimitSwitch limitTop, limitBottom;
    public MasqRotator rotator;
    public MasqElevator lift;
    public MasqServo dumper;
    public MasqCRServoSystem collector;
    public MasqVoltageSensor voltageSensor;
    public MasqClock clock;
    public MasqMotor hang;
    public MasqLimitSwitch rotateTopLimit, rotateDownLimit;
    private boolean startOpenCV = true;
    public GoldAlignDetector goldAlignDetector;
    public DogeForia dogeForia;
    public void mapHardware(HardwareMap hardwareMap) {
        voltageSensor = new MasqVoltageSensor(hardwareMap);
        dash = DashBoard.getDash();
        imu = new MasqAdafruitIMU("imu", hardwareMap);
        driveTrain = new MasqMechanumDriveTrain(hardwareMap, MasqMotorModel.ORBITAL20);
        rotator = new MasqRotator(hardwareMap);
        lift = new MasqElevator(hardwareMap);
        dumper = new MasqServo("dumper", hardwareMap);
        collector = new MasqCRServoSystem("collector", "collector2", hardwareMap);
        hang = new MasqMotor("hang", MasqMotorModel.ORBITAL20, hardwareMap);
        rotateTopLimit = new MasqLimitSwitch("limitTop", hardwareMap);
        rotateDownLimit = new MasqLimitSwitch("limitBottom", hardwareMap);
        driveTrain.resetEncoders();
        hang.setClosedLoop(true);
        hang.setKp(0.01);
        hang.setLimits(limitBottom, limitTop);
        tracker = new MasqPositionTracker(hang, rotator.rotator.motor1, imu);
        if (startOpenCV) startOpenCV(hardwareMap);
        driveTrain.setTracker(tracker);
    }
    @Override
    public MasqPIDPackage pidPackage() {
        MasqPIDPackage pidPackage = new MasqPIDPackage();
        /*-------------------------------------------------*/
        pidPackage.setKpMotorAutoLeft(0.0001);
        pidPackage.setKpMotorAutoRight(0.0001);

        pidPackage.setKpMotorTeleOpLeft(0.0001);
        pidPackage.setKpMotorTeleOpRight(0.0001);
        /*-------------------------------------------------*/
        pidPackage.setKpTurn(0.01);
        pidPackage.setKpDriveEncoder(1.5);
        pidPackage.setKpDriveAngular(0.015);
        /*-------------------------------------------------*/
        pidPackage.setKiMotorAutoLeft(0.0000);
        pidPackage.setKiMotorAutoRight(0.0000);
        pidPackage.setKiMotorTeleOpLeft(0.0000);
        pidPackage.setKiMotorTeleOpRight(0.0000);
        pidPackage.setKdMotorAutoLeft(0.0000);
        pidPackage.setKdMotorAutoRight(0.0000);
        pidPackage.setKdMotorTeleOpLeft(0.0000);
        pidPackage.setKdMotorTeleOpRight(0.0000);
        return pidPackage;
    }
    public void setStartOpenCV(boolean startOpenCV) {
        this.startOpenCV = startOpenCV;
    }
    private void startOpenCV (HardwareMap hardwareMap) {
        goldAlignDetector = new GoldAlignDetector();
        goldAlignDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 0, true);
        goldAlignDetector.useDefaults();
        goldAlignDetector.alignSize = 100;
        goldAlignDetector.alignPosOffset = 0;
        goldAlignDetector.downscale = 0.4;
        goldAlignDetector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
        goldAlignDetector.maxAreaScorer.weight = 0.005;
        goldAlignDetector.ratioScorer.weight = 5;
        goldAlignDetector.ratioScorer.perfectRatio = 1.0;
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = MasqUtils.VUFORIA_KEY;
        parameters.fillCameraMonitorViewParent = true;
        parameters.cameraName = getWebCameName(hardwareMap, "Webcam 1");
        dogeForia = new DogeForia(parameters);
        dogeForia.enableConvertFrameToBitmap();
        dogeForia.setDogeCVDetector(goldAlignDetector);
        dogeForia.enableDogeCV();
        dogeForia.showDebug();
        dogeForia.start();
    }
    public void update () {
        dash.update();
        tracker.updateSystem();
    }
    public void turnTillGold (double speed, Direction direction) {
        clock = new MasqClock();
        while (opModeIsActive() && !goldAlignDetector.getAligned() && imu.getRelativeYaw() <= 165) {
            driveTrain.setVelocity(-speed * direction.value, speed * direction.value);
            dash.create(imu.getRelativeYaw());
            dash.update();
        }
        driveTrain.setVelocity(0, 0);
    }

    public void shake(int repetition, int degree, double power) {
        for (int i = 0; i < repetition / 2; i++) {
            while (imu.getRelativeYaw() < degree) {
                driveTrain.setPower(-power, power);
            }
            while (imu.getRelativeYaw() > -degree) {
                driveTrain.setPower(power, -power);
            }
        }
        driveTrain.setPower(0);
    }

    public BlockPlacement getBlockPlacement (int block) {
        if (block > 450) return BlockPlacement.LEFT;
        else if (block > 250) return BlockPlacement.CENTER;
        else return BlockPlacement.RIGHT;
    }

}
