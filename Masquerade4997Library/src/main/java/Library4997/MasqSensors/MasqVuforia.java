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
import java.util.List;

import Library4997.MasqHardware;

/**
 * This onject assumes that target 2 and 3 are meant for tracking and target 1 i meant to stop at.
 */

public class MasqVuforia implements MasqHardware, MasqSensor{
    private String name, target1, target2, target3, asset;
    public VuforiaTrackable targetOne, targetTwo, targetThree;
    VuforiaLocalizer vuforia;
    private float mmPerInch        = 25.4f;
    private float mmBotWidth       = 18 * mmPerInch;
    private float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;
    OpenGLMatrix lastLocation = null;

    public MasqVuforia (String name, String target1, String target2, String target3, String asset){
        this.name = name;
        this.target1 = target1;
        this.target2 = target2;
        this.target3 = target3;
        this.asset = asset;
        setUp();
    }
    private void setUp () {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables target = this.vuforia.loadTrackablesFromAsset(asset);
        targetOne = target.get(0);
        targetOne.setName(target1);
        targetTwo  = target.get(1);
        targetTwo.setName(target2);
        targetThree = target.get(2);
        targetThree.setName(target3);
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(target);

        OpenGLMatrix target1Location = OpenGLMatrix
                .translation(-mmFTCFieldWidth/2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        targetOne.setLocation(target1Location);
        RobotLog.ii(this.name, target1 + "=%s", format(target1Location));

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

        ((VuforiaTrackableDefaultListener)targetOne.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)targetTwo.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)targetThree.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
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
