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
    List<OpenGLMatrix> locations = new ArrayList<>();
    OpenGLMatrix locationOne, locationTwo, locationThree, phoneLoco, lastLocation;
    private int numLocations;
    private int numTrackables;
    private int u1 = 90,
                u2 = 90,
                u3 = 90,
                v1 = 0,
                v2 = 0,
                v3 = 0,
                w1 = 90,
                w2 = 90,
                w3 = 90,
                x1,x2,x3,y1,y2,y3,z1,z2,z3;
    private List<VuforiaTrackable> trackables = new ArrayList<>();
    private List<VuforiaTrackable> allTrackables = new ArrayList<>();
    String asset;
    String targetOne, targetTwo, targetThree;
    float mmPerInch        = 25.4f;
    float mmBotWidth       = 18 * mmPerInch;
    float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;
    public MasqVuforia(String t1, String t2, String t3, int x1, int y1, int z1, int x2, int y2, int z2, int x3, int y3, int z3){
        this.x1 = x1;
        this.x2 = x2;
        this.x3 = x3;
        this.y1 = y1;
        this.y2 = y2;
        this.y3 = y3;
        this.z1 = z1;
        this.z2 = z2;
        this.z3 = z3;
        lastLocation = null;
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
        locationOne = createMatrix(x1,y1,z1,u1,v1,w1);
        locationTwo = createMatrix(x2,y2,z2,u2,v2,w2);
        locationThree = createMatrix(x3,y3,z3,u3,v3,w3);
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
    public MasqVuforia(String t1, String t2, int x1, int y1, int z1, int x2, int y2, int z2){
        this.x1 = x1;
        this.x2 = x2;
        this.y1 = y1;
        this.y2 = y2;
        this.z1 = z1;
        this.z2 = z2;
        lastLocation = null;
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
        locationOne = createMatrix(x1,y1,z1,90,0,90);
        locationTwo = createMatrix(x2,y2,z2,90,0,90);
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
    public MasqVuforia(String t1, int x1, int y1, int z1){
        this.x1 = x1;
        this.y1 = y1;
        this.z1 = z1;
        lastLocation = null;
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
        locationOne = createMatrix(x1,y1,z1,90,0,90);
        locationTwo = locationThree = null;
        locationThree = null;
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
    public void setOrientationOne(int u, int v, int w){
        locationOne = createMatrix(x1,y1,z1,u,v,w);
        locations = Arrays.asList(locationOne, locationTwo, locationThree);
        trackOne.setLocation(locationOne);
    }
    public void setOrientationTwo(int u, int v, int w){
        locationOne = createMatrix(x2,y2,z2,u,v,w);
        locations = Arrays.asList(locationOne, locationTwo, locationThree);
        trackOne.setLocation(locationOne);
    }
    public void setOrientationThree(int u, int v, int w){
        locationOne = createMatrix(x3,y3,z3,u,v,w);
        locations = Arrays.asList(locationOne, locationTwo, locationThree);
        trackOne.setLocation(locationOne);
    }
    public Boolean isSeen(String target){
        return ((VuforiaTrackableDefaultListener) getTrackable(target).getListener()).isVisible();
    }
    private VuforiaTrackable getTrackable(String target){
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
        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) getTrackable(target).getListener()).getUpdatedRobotLocation();
        if (robotLocationTransform != null) {
            lastLocation = robotLocationTransform;
        }
        return lastLocation.formatAsTransform();
    }
    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w){
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix
                        (AxesReference.EXTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES, u, v, w));
    }
    @Override
    public boolean stop() {
        return ((VuforiaTrackableDefaultListener)trackOne.getListener()).isVisible();
    }
}
