package Library4997.MasqLearning;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Archish on 9/2/17.
 */

public class Data {
    public List<Double> x = new ArrayList<>();
    public List<Double> y = new ArrayList<>();
    public void get(List e, int index){
        e.get(index);
    }
    public int index(List e, double d){
        return e.indexOf(d);
    }
    public void add(double x, double y){
        this.x.add(this.x.size(), x);
        this.y.add(this.y.size(), y);
    }
}
