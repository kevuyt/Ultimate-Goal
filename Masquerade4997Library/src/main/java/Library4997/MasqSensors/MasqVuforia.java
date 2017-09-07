package Library4997.MasqSensors;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import Library4997.MasqSensor;

/**
 * Created by Archish on 9/6/17.
 */
    //TODO CLEAN THIS CODE IT IS REALLY UGLY
public class MasqVuforia implements MasqSensor {
    VuforiaLocalizer.Parameters parameters = new
            VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
    VuforiaLocalizer vuforia;
    VuforiaTrackables vuforiaTrackables;
    VuforiaTrackable trackOne, trackTwo, trackThree;
    // redTarget = trackOne...
    List<OpenGLMatrix> locations = new ArrayList<OpenGLMatrix>();
    OpenGLMatrix locationOne, locationTwo, locationThree, phoneLoco, lastLocation;
    private int numLocations;
    private int numTrackables;
    private List<VuforiaTrackable> trackables = new ArrayList<>();
    private List<VuforiaTrackable> allTrackables = new ArrayList<>();
    String asset;
    String targetOne, targetTwo, targetThree;
    float mmPerInch        = 25.4f;
    float mmBotWidth       = 18 * mmPerInch;
    float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;
    public MasqVuforia(String t1, String t2, String t3){
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        targetOne = t1;
        targetTwo = t2;
        targetThree = t3;
        vuforiaTrackables = this.vuforia.loadTrackablesFromAsset(asset);
        trackables = Arrays.asList(trackOne, trackTwo, trackThree);
        trackOne = vuforiaTrackables.get(0);
        trackTwo = vuforiaTrackables.get(1);
        trackThree = vuforiaTrackables.get(2);
        allTrackables.addAll(vuforiaTrackables);
        numTrackables = 3;
        locationOne = OpenGLMatrix.translation(-mmFTCFieldWidth/2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        locationTwo = OpenGLMatrix.translation(-mmFTCFieldWidth/2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        locationThree = OpenGLMatrix.translation(-mmFTCFieldWidth/2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
         phoneLoco = OpenGLMatrix
                .translation(mmBotWidth/2,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 0, 0));
        numLocations = 3;
        locations = Arrays.asList(locationOne, locationTwo, locationThree);
        trackOne.setLocation(locationOne);
        trackTwo.setLocation(locationTwo);
        trackThree.setLocation(locationThree);
        ((VuforiaTrackableDefaultListener)trackOne.getListener()).setPhoneInformation(phoneLoco, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)trackTwo.getListener()).setPhoneInformation(phoneLoco, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)trackThree.getListener()).setPhoneInformation(phoneLoco, parameters.cameraDirection);
        vuforiaTrackables.activate();
    }
    public MasqVuforia(String t1, String t2){
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        targetOne = t1;
        targetTwo = t2;
        targetThree = null;
        vuforiaTrackables = this.vuforia.loadTrackablesFromAsset(asset);
        trackables = Arrays.asList(trackOne, trackTwo);
        trackOne = vuforiaTrackables.get(0);
        trackTwo = vuforiaTrackables.get(1);
        trackThree = null;
        allTrackables.addAll(vuforiaTrackables);
        numTrackables = 2;
        locationOne = OpenGLMatrix.translation(-mmFTCFieldWidth/2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        locationTwo = OpenGLMatrix.translation(-mmFTCFieldWidth/2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        locationThree = null;
        phoneLoco = OpenGLMatrix
                .translation(mmBotWidth/2,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 0, 0));
        numLocations = 3;
        locations = Arrays.asList(locationOne, locationTwo);
        trackOne.setLocation(locationOne);
        trackTwo.setLocation(locationTwo);
        ((VuforiaTrackableDefaultListener)trackOne.getListener()).setPhoneInformation(phoneLoco, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)trackTwo.getListener()).setPhoneInformation(phoneLoco, parameters.cameraDirection);
        vuforiaTrackables.activate();
    }
    public MasqVuforia(String t1){
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        targetOne = t1;
        vuforiaTrackables = this.vuforia.loadTrackablesFromAsset(asset);
        trackables = Arrays.asList(trackOne);
        trackOne = vuforiaTrackables.get(0);
        trackTwo = trackThree = null;
        allTrackables.addAll(vuforiaTrackables);
        numTrackables = 1;
        locationOne = OpenGLMatrix.translation(-mmFTCFieldWidth/2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        locationTwo = locationThree = null;
        phoneLoco = OpenGLMatrix
                .translation(mmBotWidth/2,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 0, 0));
        numLocations = 3;
        locations = Arrays.asList(locationOne);
        trackOne.setLocation(locationOne);
        ((VuforiaTrackableDefaultListener)trackOne.getListener()).setPhoneInformation(phoneLoco, parameters.cameraDirection);
        vuforiaTrackables.activate();
    }
    public void init(){
        int i = 0;
        lastLocation = null;
        for (VuforiaTrackable trackable: trackables){
            trackable = vuforiaTrackables.get(i);
            i++;
        }
        i = 0;
        for (OpenGLMatrix matrix: locations){
            matrix = OpenGLMatrix.translation(-mmFTCFieldWidth/2, 0, 0)
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.XZX,
                            AngleUnit.DEGREES, 90, 90, 0));
            trackables.get(i).setLocation(matrix);
            i++;
        }
        for (VuforiaTrackable trackable: trackables){
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLoco, parameters.cameraDirection);
        }
        phoneLoco = OpenGLMatrix
                .translation(mmBotWidth/2,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 0, 0));
        vuforiaTrackables.activate();
    }
    public Boolean isSeen(String target){
        return ((VuforiaTrackableDefaultListener)whichTrack(target).getListener()).isVisible();
    }
    private VuforiaTrackable whichTrack(String target){
        VuforiaTrackable v = null;
        if (target.equals(targetOne))
            v =  trackOne;
        else if (target.equals(targetTwo))
            v =  trackTwo;
        else if (target.equals(targetThree))
            v =  trackThree;
        return v;
    }
    public String position(String target){
        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)whichTrack(target).getListener()).getUpdatedRobotLocation();
        if (robotLocationTransform != null) {
            lastLocation = robotLocationTransform;
        }
        return lastLocation.formatAsTransform();
    }
    @Override
    public boolean stop() {
        return ((VuforiaTrackableDefaultListener)trackOne.getListener()).isVisible();
    }
}
