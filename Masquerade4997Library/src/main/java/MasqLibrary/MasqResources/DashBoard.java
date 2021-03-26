package MasqLibrary.MasqResources;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.robotcore.external.Telemetry.Log.DisplayOrder.*;


/**
 * Created by Keval Kataria on 3/15/2021
 */

public class DashBoard {
    private Telemetry telemetry;

    public DashBoard(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    public void create(String string) {telemetry.addLine(string);}
    public void create(Object data) {create(data.toString());}
    public void create(String string, Object data) {telemetry.addData(string, data);}
    public void create(Object... data) {for (Object dash : data) create(dash);}

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

    public void log(String string) {RobotLog.i(string);}
    public void log(Object data) {RobotLog.i(data.toString());}
    public void log(String string, Object data) {RobotLog.i(string, data);}

    public void setNewFirst() {telemetry.log().setDisplayOrder(NEWEST_FIRST);}
    public void setNewLast() {telemetry.log().setDisplayOrder(OLDEST_FIRST);}

    public void update() {telemetry.update();}
    public void clear(){telemetry.clearAll();}
}