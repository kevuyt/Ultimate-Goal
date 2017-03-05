package BasicLib4997;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Archish on 12/27/16.
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
        telemetry.addData(hardware.getName(), hardware.getDash());
    }

    public void createSticky(String string){
        telemetry.log().add(string);
        update();
    }
    public void createSticky(String string, Object data){
        telemetry.log().add(string, data);
        update();
    }
    public void createSticky(MasqHardware hardware) {
        telemetry.log().add(hardware.getName(), hardware.getDash());
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
