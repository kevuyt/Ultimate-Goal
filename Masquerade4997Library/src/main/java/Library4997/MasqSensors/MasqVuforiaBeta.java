package Library4997.MasqSensors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import Library4997.MasqUtilities.MasqUtils;
import Library4997.MasqUtilities.MasqHardware;
import Library4997.MasqUtilities.MasqSensor;
import Library4997.MasqWrappers.DashBoard;

/**
 * Created by Archish on 11/7/17.
 */

public class MasqVuforiaBeta implements MasqSensor, MasqHardware {
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables trackables;
    private VuforiaTrackable relicTemplate;
    private HardwareMap hardwareMap;
    public void initVuforia(HardwareMap pHardwareMap) {
        hardwareMap = pHardwareMap;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = MasqUtils.VUFORIA_KEY;

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        trackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = trackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
    }
    public void activateVuMark() {
        trackables.activate();
    }
    public String getVuMark() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        return String.format("%s", vuMark);
    }


    @Override
    public boolean stop() {
        DashBoard.getDash().create(getVuMark());
        System.out.println(getVuMark());
        DashBoard.getDash().update();
        if (getVuMark() == "LEFT" || getVuMark() == "RIGHT" || getVuMark() == "CENTER") return false;
        else return true;
    }

    @Override
    public String getName() {
        return null;
    }

    @Override
    public String[] getDash() {
        return new String[0];
    }
}
