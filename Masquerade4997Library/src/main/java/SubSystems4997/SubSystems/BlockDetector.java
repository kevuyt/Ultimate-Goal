package SubSystems4997.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqSensors.MasqREVColorSensor;

/**
 * Created by Archish on 3/25/18.
 */

public class BlockDetector {
    public MasqREVColorSensor singleBlock, doubleBlock;
    public enum Column {
        LEFT ("LEFT"),
        CENTER ("CENTER"),
        RIGHT ("RIGHT");
        public final String pos;
        Column (String pos) {this.pos = pos;}
    }
    public BlockDetector (HardwareMap hardware) {
        doubleBlock = new MasqREVColorSensor("doubleBlock", hardware);
        singleBlock = new MasqREVColorSensor("singleBlock", hardware);
    }

    public Column getColumn (Column scoredColumn) {
        if (scoredColumn == Column.CENTER) {
            if (singleBlock.isBrown() && doubleBlock.isGrey());
        }
        return Column.LEFT;
    }

}
