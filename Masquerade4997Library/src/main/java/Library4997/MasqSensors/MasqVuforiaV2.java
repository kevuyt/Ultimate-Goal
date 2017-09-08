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
 * Created by Archish on 9/7/17.
 */
//TODO Ew this is ugly FIX NOW!!!
public class MasqVuforiaV2 implements MasqSensor{
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
    VuforiaLocalizer vuforia;
    VuforiaTrackables vuforiaTrackables;
    VuforiaTrackable trackOne, trackTwo, trackThree;
    int numTargets = 0;
    List<OpenGLMatrix> locations = new ArrayList<>();
    OpenGLMatrix locationOne, locationTwo, locationThree, phoneLoco, lastLocation;
    private int u1 = 90, u2 = 90, u3 = 90,
            v1 = 0, v2 = 0, v3 = 0,
            w1 = 90, w2 = 90, w3 = 90,
            x1 = 0, x2 = 0, x3 = 0,
            y1 = 0, y2 = 0, y3 = 0,
            z1 = 0, z2 = 0,z3 = 0;
    private List<VuforiaTrackable> trackables = new ArrayList<>();
    private List<VuforiaTrackable> allTrackables = new ArrayList<>();
    String targetOne, targetTwo, targetThree;
    float mmPerInch        = 25.4f;
    float mmBotWidth       = 18 * mmPerInch;
    public MasqVuforiaV2 (String t1, String t2, String t3, String asset){
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
        numTargets = 3;
    }
    public MasqVuforiaV2 (String t1, String t2, String asset){
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        targetOne = t1;
        targetTwo = t2;
        vuforiaTrackables = this.vuforia.loadTrackablesFromAsset(asset);
        trackables = Arrays.asList(trackOne, trackTwo);
        trackOne = vuforiaTrackables.get(0);
        trackTwo = vuforiaTrackables.get(1);
        allTrackables.addAll(vuforiaTrackables);
        numTargets = 2;
    }
    public MasqVuforiaV2 (String t1, String asset){
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        targetOne = t1;
        vuforiaTrackables = this.vuforia.loadTrackablesFromAsset(asset);
        trackables = Arrays.asList(trackOne);
        trackOne = vuforiaTrackables.get(0);
        allTrackables.addAll(vuforiaTrackables);
        numTargets = 1;
    }
    public void init(){
        locationOne = createMatrix(x1, y1, z1, u1, v1, w1);
        locationTwo = createMatrix(x2, y2, z2, u2, v2, w2);
        locationThree = createMatrix(x3, y3, z3, u3, v3, w3);
        trackOne.setLocation(locationOne);
        trackTwo.setLocation(locationTwo);
        trackThree.setLocation(locationThree);
        if (numTargets <= 2){
            locationThree = null;
        }
        if (numTargets == 1){
            locationTwo = null;
        }
        phoneLoco = OpenGLMatrix
                .translation(mmBotWidth/2,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 0, 0));
        for (VuforiaTrackable trackable: trackables){
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLoco, parameters.cameraDirection);
        }
        vuforiaTrackables.activate();
    }
    public void setOrientationOne(int u, int v, int w){
        u1 = u;
        v1 = v;
        w1 = w;
    }
    public void setOrientationTwo(int u, int v, int w){
        u2 = u;
        v2 = v;
        w2 = w;
    }
    public void setOrientationThree(int u, int v, int w){
        u3 = u;
        v3 = v;
        w3 = w;
    }
    public void setPositionOne(int x, int y, int z){
        x1 = x;
        y1 = y;
        z1 = z;
    }
    public void setPositionTwo(int x, int y, int z){
        x2 = x;
        y2 = y;
        z2 = z;
    }
    public void setPositionThree(int x, int y, int z){
        x3 = x;
        y3 = y;
        z3 = z;
    }
    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w){
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix
                        (AxesReference.EXTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES, u, v, w));
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

    @Override
    public boolean stop() {
        return false;
    }
}
