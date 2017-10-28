package Library4997.MasqSensors;


import com.qualcomm.ftcrobotcontroller.R.id;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import Library4997.MasqExternal.MasqExternal;
import Library4997.MasqExternal.MasqHardware;
import Library4997.MasqRobot;
import Library4997.MasqExternal.MasqSensor;

/**
 * Created by Archish on 9/7/17.
 */
public class MasqVuforia implements MasqSensor, MasqHardware {
    private VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(id.cameraMonitorViewId);
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables vuforiaTrackables;
    private VuforiaTrackable relicTemplate;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable trackOne, trackTwo, trackThree;
    private int numTargets = 0;
    private OpenGLMatrix locationOne, locationTwo, locationThree,
            lastLocation, phoneLocation;
    private RelicRecoveryVuMark vuMark;
    private int
            u1 = 90, u2 = 90, u3 = 90,
            v1 = 0, v2 = 0, v3 = 0,
            w1 = 90, w2 = 90, w3 = 90,
            x1 = 0, x2 = 0, x3 = 0,
            y1 = 0, y2 = 0, y3 = 0,
            z1 = 0, z2 = 0,z3 = 0;
    private List<VuforiaTrackable> trackables = new ArrayList<>();
    private List<VuforiaTrackable> allTrackables = new ArrayList<>();
    private String targetOne, targetTwo, targetThree;
    private float mmPerInch = 25.4f;
    private float mmBotWidth = 18 * mmPerInch;
    public enum Facing {
        RIGHT (new int[]{90,0,90}),
        LEFT (new int[]{90,0,-90}),
        BACK (new int[]{90,0,0}),
        FOREWORD (new int[]{-90,0,0});
        public final int[] value;
        Facing(int[] value) {this.value = value;}
    }
    public MasqVuforia(String t1, String t2, String t3, String asset) {
        parameters.vuforiaLicenseKey = MasqExternal.VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        targetOne = t1;
        targetTwo = t2;
        targetThree = t3;
        vuforiaTrackables = this.vuforia.loadTrackablesFromAsset(asset);
        trackOne = vuforiaTrackables.get(0);
        trackTwo = vuforiaTrackables.get(1);
        trackThree = vuforiaTrackables.get(2);
        trackables = Arrays.asList(trackOne, trackTwo, trackThree);
        trackOne.setName(targetOne);
        trackTwo.setName(targetTwo);
        trackThree.setName(targetThree);
        allTrackables.addAll(vuforiaTrackables);
        numTargets = 3;
    }
    public MasqVuforia(String t1, String t2, String asset)  {
        parameters.vuforiaLicenseKey = MasqExternal.VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        targetOne = t1;
        targetTwo = t2;
        vuforiaTrackables = this.vuforia.loadTrackablesFromAsset(asset);
        trackOne = vuforiaTrackables.get(0);
        trackTwo = vuforiaTrackables.get(1);
        trackables = Arrays.asList(trackOne, trackTwo);
        trackOne.setName(targetOne);
        trackTwo.setName(targetTwo);
        allTrackables.addAll(vuforiaTrackables);
        numTargets = 2;
    }
    public MasqVuforia(String t1, String asset) {
        parameters.vuforiaLicenseKey = MasqExternal.VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        targetOne = t1;
        vuforiaTrackables = this.vuforia.loadTrackablesFromAsset(asset);
        trackOne = vuforiaTrackables.get(0);
        trackables = Arrays.asList(trackOne);
        trackOne.setName(targetOne);
        allTrackables.addAll(vuforiaTrackables);
        numTargets = 1;
    }
    public MasqVuforia(){}
    public void init(){
        locationOne = createMatrix(x1, y1, z1, u1, v1, w1);
        locationTwo = createMatrix(x2, y2, z2, u2, v2, w2);
        locationThree = createMatrix(x3, y3, z3, u3, v3, w3);
        if (numTargets == 1) {
            trackOne.setLocation(locationOne);
            locationTwo = null;
            locationThree = null;
        } else if (numTargets == 2) {
            trackOne.setLocation(locationOne);
            trackTwo.setLocation(locationTwo);
            locationThree = null;
        } else if (numTargets == 3) {
            trackOne.setLocation(locationOne);
            trackTwo.setLocation(locationTwo);
            trackThree.setLocation(locationThree);
        }
        phoneLocation = OpenGLMatrix.translation(mmBotWidth/2,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 0, 0));
        for (VuforiaTrackable trackable: trackables){
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocation, parameters.cameraDirection);
        }
        loadVuMark(trackOne);
        vuforiaTrackables.activate();
    }
    private void loadVuMark (VuforiaTrackable trackable){vuMark = RelicRecoveryVuMark.from(trackable);}
    public String getTrackable() {return String.valueOf(vuMark);}
    public void initVuMark(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = MasqExternal.VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
    }
    public void activateVuMark(){
        relicTrackables.activate();
    }
    public String getVuMark () {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) return String.format("%s", vuMark);
        else return "UNKNOWN";
    }
    private void setOrientationOne(int u, int v, int w){u1 = u; v1 = v; w1 = w;}
    private void setOrientationTwo(int u, int v, int w){u2 = u; v2 = v; w2 = w;}
    private void setOrientationThree(int u, int v, int w){u3 = u; v3 = v; w3 = w;}
    public void setOrientationOne(Facing t){setOrientationOne(t.value[0], t.value[1], t.value[2]);}
    public void setOrientationTwo(Facing t){setOrientationTwo(t.value[0], t.value[1], t.value[2]);}
    public void setOrientationThree(Facing t){setOrientationThree(t.value[0], t.value[1], t.value[2]);}
    public void setPositionOne(int x, int y, int z){x1 = x; y1 = y; z1 = z;}
    public void setPositionTwo(int x, int y, int z){x2 = x; y2 = y; z2 = z;}
    public void setPositionThree(int x, int y, int z){x3 = x; y3 = y; z3 = z;}
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
    private String position(String target){
        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) getTrackable(target).getListener()).getUpdatedRobotLocation();
        if (robotLocationTransform != null) {
            lastLocation = robotLocationTransform;
        }
        if (lastLocation == null) {
            return "Position UNKNOWN";
        }
        return lastLocation.formatAsTransform();
    }
    private boolean isSeen(String track){return ((VuforiaTrackableDefaultListener)getTrackable(track).getListener()).isVisible();}
    public String position(MasqRobot.Targets target){
        return position(target.value);
    }
    public boolean isSeen(MasqRobot.Targets track){
        return isSeen(track.value);
    }
    @Override
    public boolean stop() {
        if (getVuMark() == "UNKNOWN") return true;
        else return false;
    }
    @Override
    public String getName() {return "VUFORIA";}
    public String[] getDash() {
        return new String[]{
                getName() +
                "TargetOneSeen" + Boolean.toString(isSeen(targetOne)),
                "TargetTwoSeen" + Boolean.toString(isSeen(targetTwo)),
                "TargetThreeSeen" + Boolean.toString(isSeen(targetThree)),
                "TargetOnePosition" + position(targetOne),
                "TargetTwoPosition" + position(targetTwo),
                "TargetThreePosition" + position(targetThree),
        };
    }
}
