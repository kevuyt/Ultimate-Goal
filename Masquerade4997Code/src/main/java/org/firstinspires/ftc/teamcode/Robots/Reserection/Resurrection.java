package org.firstinspires.ftc.teamcode.Robots.Reserection;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.DogeForia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Robots.Reserection.Resources.BlockPlacement;
import org.firstinspires.ftc.teamcode.Robots.Reserection.ResurrectionSubSystems.MasqCollectionLift;
import org.firstinspires.ftc.teamcode.Robots.Reserection.ResurrectionSubSystems.MasqScoreLift;

import Library4997.MasqControlSystems.MasqPID.MasqPIDPackage;
import Library4997.MasqControlSystems.MasqPurePursuit.MasqPositionTracker;
import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqResources.MasqUtils;
import Library4997.MasqRobot;
import Library4997.MasqSensors.MasqAdafruitIMU;
import Library4997.MasqSensors.MasqLimitSwitch;
import Library4997.MasqServos.MasqCRServoSystem;
import Library4997.MasqServos.MasqServo;
import Library4997.MasqWrappers.DashBoard;

/**
 * Created by Archishmaan Peyyety on 6/2/18.
 * Project: MasqLib
 */

public class Resurrection extends MasqRobot {
    /*---------------------Hardware-------------------------*/
    public MasqAdafruitIMU imu;
    public MasqMotor collectorDumper;
    public MasqServo particleDumper;
    public MasqLimitSwitch rotateTopSwitch;
    public MasqCRServoSystem collector;
    public MasqMotor hang;
    public MasqLimitSwitch hangTopSwitch, hangBottomSwitch, collectionLiftSwitch, collectionDumpTopSwitch;
    public MasqCollectionLift collectionLift;
    public MasqScoreLift scoreLift;
    /*---------------------OPEN-CV---------------------------*/
    private boolean startOpenCV = true;
    public GoldAlignDetector goldAlignDetector;
    public DogeForia dogeForia;
    public void mapHardware(HardwareMap hardwareMap) {
        dash = DashBoard.getDash();
        imu = new MasqAdafruitIMU("imu", hardwareMap);
        driveTrain = new MasqMechanumDriveTrain(hardwareMap, MasqMotorModel.ORBITAL20);


        collectionLift = new MasqCollectionLift("collectLift", hardwareMap);
        scoreLift = new MasqScoreLift("scoreLift", hardwareMap);
        collectorDumper = new MasqMotor("collectorDumper", MasqMotorModel.NEVEREST40, hardwareMap);

        particleDumper = new MasqServo("dumper", hardwareMap);
        collector = new MasqCRServoSystem("collector", "collector2", hardwareMap);
        hang = new MasqMotor("hang", MasqMotorModel.ORBITAL20, hardwareMap);

        rotateTopSwitch = new MasqLimitSwitch("rotTop", hardwareMap);
        hangTopSwitch = new MasqLimitSwitch("limitTop", hardwareMap);
        hangBottomSwitch = new MasqLimitSwitch("limitBottom", hardwareMap);
        collectionDumpTopSwitch = new MasqLimitSwitch("collectTop", hardwareMap);
        collectionLiftSwitch = new MasqLimitSwitch("collectLiftSwitch", hardwareMap);

        tracker = new MasqPositionTracker(hang, collectionLift.lift, imu);
        if (startOpenCV) startOpenCV(hardwareMap);
        hang.setClosedLoop(true);
        hang.setKp(0.01);
        driveTrain.setTracker(tracker);
        driveTrain.resetEncoders();
    }
    @Override
    public MasqPIDPackage pidPackage() {
        MasqPIDPackage pidPackage = new MasqPIDPackage();
        /*-------------------------------------------------*/
        pidPackage.setKpMotorAutoLeft(0.001);
        pidPackage.setKpMotorAutoRight(0.001);

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

    public BlockPlacement getBlockPlacement (int block) {
        if (block > 450) return BlockPlacement.LEFT;
        else if (block > 250) return BlockPlacement.CENTER;
        else return BlockPlacement.RIGHT;
    }

}
