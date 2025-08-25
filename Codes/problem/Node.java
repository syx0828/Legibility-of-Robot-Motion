package problem;
import java.util.List;

public class Node {
    private int nIdx;
    private int parent;
    private ArmConfig config;
    private List<ArmConfig> path;

    public Node(int nIdx, ArmConfig config) {
        this.nIdx = nIdx;
        this.parent = -1;
        this.config = config;
        this.path = null;
    }

    

    public int getnIdx() {
        return this.nIdx;
    }

    public int getParent() {
        return this.parent;
    }

    public void setParent(int parentIdx) {
        this.parent = parentIdx;
    }

    public ArmConfig getConfig() {
        return this.config;
    }

    public List<ArmConfig> getPath() {
        return this.path;
    }

    public void setPath(List<ArmConfig> path) {
        this.path = path;
    }

    public double distance(Node a) {
        double dist = this.config.getBaseCenter().distance(a.config.getBaseCenter());
        return dist;
    }
}
