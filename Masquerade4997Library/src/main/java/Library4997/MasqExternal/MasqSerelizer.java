package Library4997.MasqExternal;

import android.content.Context;

import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;

/**
 * Created by Archish on 10/15/17.
 */

public class MasqSerelizer implements Runnable {
    private boolean close = false;
    private MasqHardware hardware;
    public void startFileWriting(){
        new Thread(this).start();
    }
    public void setHardwareTracking(MasqHardware hardware){
        this.hardware = hardware;
    }
    private void createFile(){

    }
    private void writeToFile(){

    }
    public String[] readFromFile(){

    }
    private void update () {

    }
    @Override
    public void run() {
        boolean close = false;
        while (!close) {
            update();
            close = this.close;
            sleep();
        }
    }
    public void close() {close = true;}
    private void sleep(){
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
