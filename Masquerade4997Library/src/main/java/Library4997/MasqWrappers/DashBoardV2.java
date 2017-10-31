package Library4997.MasqWrappers;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

import Library4997.MasqExternal.MasqExternal;
import Library4997.MasqExternal.MasqHardware;

/**
 * Created by Archish on 10/31/17.
 */

public class DashBoardV2{
    public List<Telemetry.Item> items = new ArrayList<>();
    private int dashLength;
    private Telemetry telemetry;
    private boolean close = false;
    public DashBoardV2(Telemetry telemetry){
        this.telemetry  = telemetry;
        instance = this;
    }

    public static DashBoardV2 getDash(){return instance;}
    public static DashBoardV2 instance;

    public void create(String string) {
        items.add(telemetry.addData(string,""));
    }
    public void create(Object data) {
        telemetry.addLine(data.toString());
    }
    public void create(String string, Object data) {
        telemetry.addData(string, data);
    }
    public void create(final MasqHardware hardware) {
        dashLength = hardware.getDash().length;
        for (int i = 0; i < dashLength; i++) {
            telemetry.addData(hardware.getName(), hardware.getDash()[i]);
        }
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
    public void update() {
        telemetry.update();
    }
    public void clear(){telemetry.clearAll();}
    public void setController(final MasqController masqController1, final MasqController masqController2) {
        Runnable updateCont1 = new Runnable() {
            @Override
            public void run() {
                masqController1.update();
            }
        };
        Runnable updateCont2 = new Runnable() {
            @Override
            public void run() {
                masqController2.update();
            }
        };
        Runnable update = new Runnable() {
            @Override
            public void run() {
                telemetry.update();
            }
        };
        telemetry.addAction(updateCont1);
        telemetry.addAction(updateCont2);
        telemetry.addAction(update);
    }


}
