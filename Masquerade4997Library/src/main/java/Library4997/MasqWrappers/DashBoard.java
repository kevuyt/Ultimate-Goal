package Library4997.MasqWrappers;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Library4997.MasqUtilities.MasqHardware;
import Library4997.MasqUtilities.MasqUtils;
import SubSystems4997.MasqSubSystem;

/**
 * This is a telemetry wrapper class.
 * It provides additional functionality such as stickied messages.
 * It supports multiple types of inputs, including MasqHardware objects.
 */

public class DashBoard {
    private int dashLength;
    private Telemetry telemetry;
    private boolean open;
    public DashBoard(Telemetry telemetry){
        this.telemetry  = telemetry;
        instance = this;
    }

    public static DashBoard getDash(){return instance;}
    public static DashBoard instance;

    public void create(String string) {
        telemetry.addLine(string);
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
    public void create(final MasqSubSystem subSystem) {
        create(subSystem.getName());
        for (MasqHardware hardware : subSystem.getComponents()) {
            create(hardware.getDash());
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
    public void createSticky(final MasqSubSystem subSystem) {
        for (MasqHardware hardware : subSystem.getComponents()) {
            createSticky(hardware.getDash());
        }
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
    public void clear(){
        telemetry.clearAll();
    }
    public void close () {
        open = false;
    }
    public void open () {
        open = true;
    }
    public void startUpdate (){
        Runnable main = new Runnable() {
            @Override
            public void run() {
                while (MasqUtils.opModeIsActive() && open) {
                    update();
                    MasqUtils.sleep(100);
                }
            }
        };
        Thread t = new Thread(main);
        t.start();
    }
}
