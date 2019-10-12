package Library4997.MasqResources;

import android.annotation.SuppressLint;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;

public class MasqSerializer {
    private Writer writer;
    private StringBuffer lineBuffer;
    private long msBase;
    private long nsBase;

    public MasqSerializer (String fileName) {
        @SuppressLint("SdCardPath") String directoryPath    = "/sdcard/FIRST/DataLogger";
        String filePath         = directoryPath + "/" + fileName + ".csv";
        new File(directoryPath).mkdir();
        try {
            writer = new FileWriter(filePath);
            lineBuffer = new StringBuffer(128);
        } catch (IOException e) {
        }
        msBase = System.currentTimeMillis();
        nsBase = System.nanoTime();
        addField("sec");
        addField("d ms");
    }

    @SuppressLint("DefaultLocale")
    private void flushLineBuffer(){
        long milliTime,nanoTime;

        try {
            lineBuffer.append('\n');
            writer.write(lineBuffer.toString());
            lineBuffer.setLength(0);
        }
        catch (IOException ignored){
        }
        milliTime   = System.currentTimeMillis();
        nanoTime    = System.nanoTime();
        addField(String.format("%.3f",(milliTime - msBase) / 1.0E3));
        addField(String.format("%.3f",(nanoTime - nsBase) / 1.0E6));
        nsBase      = nanoTime;
    }

    public void closeDataLogger() {
        try {
            writer.close();
        }
        catch (IOException e) {
        }
    }

    public void addField(String s) {
        if (lineBuffer.length()>0) {
            lineBuffer.append(',');
        }
        lineBuffer.append(s);
    }

    public void addField(char c) {
        if (lineBuffer.length()>0) {
            lineBuffer.append(',');
        }
        lineBuffer.append(c);
    }

    public void addField(boolean b) {
        addField(b ? '1' : '0');
    }

    public void addField(byte b) {
        addField(Byte.toString(b));
    }

    public void addField(short s) {
        addField(Short.toString(s));
    }

    public void addField(long l) {
        addField(Long.toString(l));
    }

    public void addField(float f) {
        addField(Float.toString(f));
    }

    public void addField(double d) {
        addField(Double.toString(d));
    }

    public void newLine() {
        flushLineBuffer();
    }

    @Override
    protected void finalize() throws Throwable {
        closeDataLogger();
        super.finalize();
    }
}