package Library4997;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Custom Telemetry
 */

public class DashBoard {

    public DashBoard(org.firstinspires.ftc.robotcore.external.Telemetry telemetry){
        this.telemetry  = telemetry;
        instance = this;
    }

    public static DashBoard getDash(){
        return instance;
    }
    public static DashBoard instance;
    private org.firstinspires.ftc.robotcore.external.Telemetry telemetry;
    public void create(String string) {
        telemetry.addLine(string);
    }
    public void create(String string, Object data) {
        telemetry.addData(string, data);
    }
    public void create(final MasqHardware hardware) {
        int dashLength = hardware.getDash().length;
        for (int i = 0; i < dashLength; i++) {
            telemetry.addData(hardware.getName(), hardware.getDash()[i]);
        }
    }

    public void createSticky(String string){
        telemetry.log().add(string);
        update();
    }
    public void createSticky(String string, Object data){
        telemetry.log().add(string, data);
        update();
    }
    public void createSticky(final MasqHardware hardware) {
        int dashLength = hardware.getDash().length;
        for (int i = 0; i < dashLength; i ++) {
            telemetry.log().add(hardware.getName(), hardware.getDash()[i]);
        }
        update();
    }
    public void setNewFirst() {
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
    }
    public void setNewLast() {
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.OLDEST_FIRST);
    }
    public void update () {
        telemetry.update();
    }


}
