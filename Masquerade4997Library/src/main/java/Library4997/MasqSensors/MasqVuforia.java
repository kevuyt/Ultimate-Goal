package Library4997.MasqSensors;

import com.qualcomm.robotcore.util.RobotLog;

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
import java.util.Collections;
import java.util.List;

import Library4997.MasqHardware;
import Library4997.MasqSensor;

/**
 * This onject assumes that target 2 and 3 are meant for tracking and target 1 i meant to stop at.
 */

public class MasqVuforia implements MasqHardware, MasqSensor {
    private String name, target1, target2, target3, asset;
    public VuforiaTrackable targetOne, targetTwo, targetThree;
    private List<VuforiaTrackable> trackables;
    private List<String> names;
    private List<Integer> trackableCount;
    private VuforiaLocalizer vuforia;
    private float mmPerInch        = 25.4f;
    private float mmBotWidth       = 18 * mmPerInch;
    private float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;
    OpenGLMatrix lastLocation = null;

    public MasqVuforia (String target1, String target2, String target3, String asset){
        this.target1 = target1;
        this.target2 = target2;
        this.target3 = target3;
        this.asset = asset;
        trackables = Arrays.asList(targetOne, targetTwo, targetThree);
        names = Arrays.asList(target1, target2, target3);
        trackableCount = Arrays.asList(1,2,3);
        setUp();
    }
    public MasqVuforia (String target1, String target2, String asset){
        this.target1 = target1;
        this.target2 = target2;
        this.asset = asset;
        trackables = Arrays.asList(targetOne, targetTwo);
        names = Arrays.asList(target1, target2);
        trackableCount = Arrays.asList(1,2);
        setUp();
    }
    public MasqVuforia (String target1, String asset){
        this.target1 = target1;
        this.asset = asset;
        trackables = Collections.singletonList(targetOne);
        names = Collections.singletonList(target1);
        trackableCount = Collections.singletonList(1);
        setUp();
    }
    private void setUp () {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables target = this.vuforia.loadTrackablesFromAsset(asset);
        for (VuforiaTrackable trackable: trackables) {
            int i = 0;
            trackable = target.get(i);
            trackable.setName(names.get(i));
        }

        OpenGLMatrix target2Location = OpenGLMatrix
                .translation(0, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        targetTwo.setLocation(target2Location);
        RobotLog.ii(this.name, target2 + "=%s", format(target2Location));

        OpenGLMatrix target3Location = OpenGLMatrix
                .translation(0, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        targetThree.setLocation(target3Location);
        RobotLog.ii(this.name, target3 + "=%s", format(target3Location));

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(mmBotWidth/2,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 0, 0));
        RobotLog.ii(this.name, "phone=%s", format(phoneLocationOnRobot));

        for (VuforiaTrackable vuforiaTrackable: trackables){
            ((VuforiaTrackableDefaultListener)vuforiaTrackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }
        trackables.addAll(target);
    }
    public boolean isSeen (VuforiaTrackable trackable) {
        return ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible();
    }
    public void createTarget () {

    }
    public String position(VuforiaTrackable trackable) {
            String position;
            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
            }
            if (lastLocation != null) {
                position = format(lastLocation);
            } else {
                position =  "Pos" + "Unknown";
            }
            return position;
    }
    private String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }


    public String getName() {
        return name;
    }
    public String[] getDash() {
        return new String[]{
                name +
                "TargetOneSeen" + Boolean.toString(isSeen(targetOne)),
                "TargetTwoSeen" + Boolean.toString(isSeen(targetTwo)),
                "TargetThreeSeen" + Boolean.toString(isSeen(targetThree)),
                "TargetOnePosition" + position(targetOne),
                "TargetTwoPosition" + position(targetTwo),
                "TargetThreePosition" + position(targetThree),
        };
    }
    public boolean stop() {
        return isSeen(targetOne);
    }
}
