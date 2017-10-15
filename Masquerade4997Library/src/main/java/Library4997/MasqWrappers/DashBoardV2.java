package Library4997.MasqWrappers;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.LinkedHashMap;

import Library4997.MasqExternal.MasqExceptions.DashBoardException;
import Library4997.MasqExternal.MasqHardware;

/**
 * Created by Archish on 10/14/17.
 */

public class DashBoardV2 {
    private int dashLength;
    private Telemetry telemetry;
    private LinkedHashMap<String, Object> dasboard = new LinkedHashMap<>();
    public DashBoardV2(Telemetry telemetry){
        this.telemetry  = telemetry;
        instance = this;
    }

    public static DashBoardV2 getDash(){return instance;}
    public static DashBoardV2 instance;

    public void create(String string) {
        telemetry.addLine(string);
        update();
    }
    public void create(Object data) {
        telemetry.addLine(data.toString());
        update();
    }
    public void create(String string, Object data) {
        telemetry.addData(string, data);
        update();
    }
    public void create(final MasqHardware hardware) {
        if (hardware.getDash() != null) {
            dashLength = hardware.getDash().length;
            for (int i = 0; i < dashLength; i++) {
                telemetry.addData(hardware.getName(), hardware.getDash()[i]);
            }
        } else throwException(hardware);
        update();
    }

    public void createSticky(String string){
        telemetry.log().add(string);
        update();
    }
    public void createSticky(Object data){
        telemetry.log().add(data.toString());
        update();
    }
    public void createSticky(String string, Object data){
        telemetry.log().add(string, data);
        update();
    }
    public void createSticky(final MasqHardware hardware) {
        dashLength = hardware.getDash().length;
        for (int i = 0; i < dashLength; i ++) {
            telemetry.log().add(hardware.getName(), hardware.getDash()[i]);
        }
        update();
    }

    public void log(String string){
        RobotLog.i(string);
    }
    public void log(Object data){
        RobotLog.i(data.toString());
    }
    public void log(String string, Object data){
        RobotLog.i(string, data);
    }
    public void log(final MasqHardware hardware) {
        dashLength = hardware.getDash().length;
        for (int i = 0; i < dashLength; i ++) {
            RobotLog.i(hardware.getName(), hardware.getDash()[i]);
        }
    }

    public void setNewFirst() {
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
    }
    public void setNewLast() {
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.OLDEST_FIRST);
    }
    private void update () {
        telemetry.update();
    }
    private void throwException(MasqHardware hardware){
        try {
            throw new DashBoardException(hardware);
        } catch (DashBoardException e) {
            e.printStackTrace();
        }
    }
}
