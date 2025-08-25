import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Random;

import javax.sound.sampled.Line;

import problem.ArmConfig;
import problem.Node;
import problem.Obstacle;
import problem.ProblemSpec;
import tester.Tester;

import java.awt.Point;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;

public class greedyLegOptTest {
    /** Existing vertices of graph that will be used to sesearch for the path */
    private static List<Node> vertices = new ArrayList<>();
    /** Subregions and sampling density for each sub-region (used for adaSample only)*/
    private static HashMap<Point2D,Double> regions = new HashMap<>();
    /** Loaded problem specification */
    private static ProblemSpec ps;
    /** Get obstacle information */
    private static List<Obstacle> obstacles;
    /** Validity tester */
    private static Tester tester = new Tester();
    /** The robot has gripper or not */
    private static boolean hasGripper;


    /**
     * Initialize setting
     * 
     * @param probFile
     *      input problem specification file
     * 
     * @throws IOException
     */
    public static void initialize(String probFile, String graphFile) throws IOException{
        try {
            // Get necessary problem settings
            ps = new ProblemSpec();
            ps.loadProblem(probFile);
            obstacles = ps.getObstacles();
            hasGripper = ps.getInitialState().hasGripper();
        } catch (IOException e) {
            System.out.println("IO Exception occured");
        }

        // Construct new maps
        // Initialize map
        vertices.clear();
        Node initNode = new Node(0, ps.getInitialState());
        Node goalNode = new Node(1, ps.getGoalState());
        vertices.add(initNode);
        vertices.add(goalNode);
        // Initialize sub-regions and sample densities
        for (int y = 0; y < 4; y++) {
            for (int x = 0; x < 4; x++) {
                Point2D region = new Point2D.Double(x * 0.25, y * 0.25);
                regions.put(region, 1.0);
            }
        }
        // Call constructor function
        createGraph(graphFile);
        /*
        // Read pre-constructed prm graph
        
        try (BufferedReader reader = new BufferedReader(new FileReader(graphFile))) {
            String line;
            int idx = 0;

            while ((line = reader.readLine()) != null) {
                String[] configs = line.split(" ");

                // Extract base config
                double x = Double.parseDouble(configs[0]);
                double y = Double.parseDouble(configs[1]);
                Point2D baseConfig = new Point2D.Double(x, y);
                // Extract joints config
                List<Double> jointConfig = new ArrayList<>();
                for (int i = 2; i < ps.getJointCount() + 2; i++) {
                    jointConfig.add(Double.parseDouble(configs[i]));
                }

                // Construct new config
                ArmConfig armConfig = new ArmConfig(baseConfig, jointConfig);
                // Construct new node
                Node node = new Node(idx, armConfig);

                // Add new vertex onto graph
                vertices.add(node);
            }
           

        } catch (IOException e) {
            e.printStackTrace();
        }
        */
        System.out.println(vertices.size());
    }


    /**
     * Adaptively sample points for robot configuration,
     * add collision-free points as vertices of the graph
     * 
     * @param n
     *      number of points to be sampled for each sub-region in each round
     */
    public static void adaSample(int n) {
        // Sample n points for each round
        Random r = new Random();

        for (Point2D region : regions.keySet()) {
            double density = regions.get(region);
            int amount = (int)Math.ceil(density * n);
            int fails = 0;
            for (int i = 0; i < amount; i++) {
                // Generate base location info
                double x = r.nextDouble() * 0.25 + region.getX();
                double y = r.nextDouble() * 0.25 + region.getY();
                Point2D location = new Point2D.Double(x, y);

                // Generate joints angels info
                int jointCount = ps.getJointCount();
                List<Double> joints = new ArrayList<>();
                for (int j = 0; j < jointCount; j++) {
                    double LimitRadAngle =  Math.toRadians(150);
                    double a = -LimitRadAngle + (LimitRadAngle - -LimitRadAngle) * r.nextDouble();
                    joints.add(a);
                }

                // Form new robot configuration without gripper
                ArmConfig ac = new ArmConfig(location, joints);

                // Generate gripper lengths info
                if(hasGripper){
                    List<Double> griLens = new ArrayList<>();
                    for (int j = 0; j < 4; j++) {
                        double a = 0.03 + (0.07 - 0.03) * r.nextDouble();
                        griLens.add(a);
                    }
                    // Update robot configuration with gripper
                    ac = new ArmConfig(location, joints, griLens);
                }

                // Check if the config is collision-free, and so add onto the graph
                if (!tester.hasCollision(ac, obstacles)  && !tester.hasSelfCollision(ac) && tester.fitsBounds(ac)) {
                    int idx = vertices.size();
                    Node vertex = new Node(idx, ac);
                    vertices.add(vertex);
                } else {
                    fails++;
                }
            }

            if (fails == 0) {regions.put(region, density+=0.5);}
            else {regions.put(region, Math.exp(-0.2 * fails));}
        }

        //System.out.println("finish samples (ada)");
    }


    /**
     * Create a graph with no edges and about 100 vertices
     */
    public static void createGraph(String graphFile) {
        // Adaptively sample no more than 100 vertices onto graph
        while (vertices.size() < 135) {
            adaSample(10);
        }

        File outputFile = new File(graphFile);
        try {
            BufferedWriter bw = new BufferedWriter(new FileWriter(outputFile));
            // Add vertices
            for (Node v : vertices) {
                bw.write(v.getConfig().toString());
                bw.newLine();
            }
            bw.close();
        } catch (IOException e){
            e.printStackTrace();
        }
    }


    /**
     * Check collision for a line segment between two configurations.
     * The transformation will be broke down and completed with primitives steps,
     * where on each step the transformation won't exceed the robot limitation.
     * The line segment (path) from one config to another is valid if no collision occurs
     * at any primitive steps, otherwise is invalid.
     * 
     * @param parentNode
     *      the start vertex on the graph
     * @param endNode
     *      the vertex on the other 
     * 
     * @return
     *      true if the line segment collide with any obstacles
     */
    public static boolean makePath(int parent, int child) {
        boolean collide = false;
        List<ArmConfig> path = new ArrayList<>();
        Node parentNode = vertices.get(parent);
        Node childNode = vertices.get(child);

        // Get end points' configurations
        ArmConfig parentConfig = parentNode.getConfig();
        ArmConfig childConfig = childNode.getConfig();
        // Get end points' base positions
        Point2D parentBase = parentConfig.getBaseCenter();
        Point2D childBase = childConfig.getBaseCenter();
        // Get end points' arm angles
        List<Double> parentAngle = parentConfig.getJointAngles();
        List<Double> childAngle = childConfig.getJointAngles();
        int jc = parentConfig.getJointCount();
        // Get end points' gripper lengths
        List<Double> parentGriLen = null;
        List<Double> childGriLen = null;

        // Count the max number of steps the robot needs to take the transformation
        // Consider the arm transformation
        int armSteps = (int)Math.ceil(parentConfig.maxAngleDiff(childConfig) / Math.toRadians(0.1));
        // Consider the base transformation
        int baseStep = (int)Math.ceil(parentBase.distance(childBase) / 0.001);
        // Consider the gripper transformation(if has)
        int gripperstep = 0;
        if (hasGripper){
            gripperstep = (int)Math.ceil(parentConfig.maxGripperDiff(childConfig) / 0.001);
            // Update end points' gripper lengths
            parentGriLen = parentConfig.getGripperLengths();
            childGriLen = childConfig.getGripperLengths();
        }

        int maxStep = 0;
        if (armSteps < baseStep) {maxStep = baseStep;}
        if (maxStep < gripperstep) {maxStep = gripperstep;}
        if (maxStep < armSteps) {maxStep = armSteps;}

        // Calculate the max transformation of arm and base per step
        List<Double> stepAngles = new ArrayList<>();
        for (int j = 0; j < jc; j++) {
            double angleDiff = childAngle.get(j) - parentAngle.get(j);
            double stepAngle = angleDiff / maxStep;
            stepAngles.add(stepAngle);
        }
        Double stepMoveX = (childBase.getX() - parentBase.getX()) / maxStep;
        Double stepMoveY = (childBase.getY() - parentBase.getY()) / maxStep;
        
        List<Double> stepGriLens = new ArrayList<>();
        if(hasGripper){
            for (int j = 0; j < 4; j++) {
                double gripperDiff = childGriLen.get(j) - parentGriLen.get(j);
                double stepGriLen = gripperDiff / maxStep;
                stepGriLens.add(stepGriLen);
            }
        }

        // Check collision on each primitive step
        // Note that the start and end configurations are already known as valid
        for (int s = 1; s < maxStep; s++) {
            // Construct configuration on each primitive step
            double transX = childBase.getX() - s * stepMoveX;
            double transY = childBase.getY() - s * stepMoveY;
            Point2D transBase = new Point2D.Double(transX, transY);
            List<Double> transAngles = new ArrayList<>();
            for (int j = 0; j < jc; j++) {
                double transAngle = childAngle.get(j) - s * stepAngles.get(j);
                transAngles.add(transAngle);
            }

            ArmConfig transConfig = new ArmConfig(transBase, transAngles);

            if (hasGripper){
                List<Double> transGriLens = new ArrayList<>();
                for (int j = 0; j < 4; j++) {
                    double transGriLen = childGriLen.get(j) - s * stepGriLens.get(j);
                    transGriLens.add(transGriLen);
                }

                transConfig = new ArmConfig(transBase, transAngles, transGriLens);
            }
            // Stop checking if collision occurs on any one of the primitive steps
            if (tester.hasCollision(transConfig, obstacles) || tester.hasSelfCollision(transConfig) || !tester.fitsBounds(transConfig)) {
                collide = true;
                break;
            }
            path.add(transConfig);
        }

        if (collide == false) {
            childNode.setPath(path);
            vertices.set(child, childNode);
        }

        return collide;
    } 



     /**
     * Calculate the cost between two nodes by taking l2 norm
     * 
     * @param startNode
     *      the start node of the trajectory
     * @param endNode
     *      the end node of the trajectory
     * 
     * @return
     *      cost of trajectory between two nodes
     */
    public static double cost(Node startNode, Node endNode) {
        // Get the configs
        ArmConfig startConfig = startNode.getConfig();
        ArmConfig endConfig = endNode.getConfig();
        // Extract config values of base and joints
        String[] startVector = startConfig.toString().split(" ");
        String[] endVector = endConfig.toString().split(" ");
        
        // Take the l2 norm
        double sum = 0;
        for (int i = 0; i < startVector.length; i++) {
            sum += Math.pow((Double.parseDouble(startVector[i]) - Double.parseDouble(endVector[i])), 2);
        }
        sum = Math.sqrt(sum);
        
        return sum;
    }


     /**
     * Calculate the cost of the optimal trajectory (minimum cost) to the goal node
     * (The goal node is problem-depend) 
     * 
     * @param startNode
     *      the start vertex on the graph
     * @param endNode
     *      the vertex on the other 
     * 
     * @return
     *      cost of optimal trajectory (minimum cost) to goal node
     */
    public static double bestCost(Node startNode, Node endNode) {
        // Construct positions for polygon's vertices in visibility graph
        // NOTE: These points are problem dependent. Need to be changed if using another graph
        Point2D startPos = startNode.getConfig().getBaseCenter();
        Point2D endPos = endNode.getConfig().getBaseCenter();
        // [Four criticle points] are used for [2 obstacles]
        Point2D p1Pos = new Point2D.Double(0.4, 0.4);
        Point2D p2Pos = new Point2D.Double(0.6, 0.4);
        Point2D p3Pos = new Point2D.Double(0.4, 0.6);
        Point2D p4Pos = new Point2D.Double(0.6, 0.6);
        // [Six criticle points] are used for [3 obstacles]
        Point2D p5Pos = new Point2D.Double(0.2, 0.4);
        Point2D p6Pos = new Point2D.Double(0.2, 0.6);
        // [Eight criticle points] are used for [4 obstacles]
        Point2D p7Pos = new Point2D.Double(0.8, 0.4);
        Point2D p8Pos = new Point2D.Double(0.8, 0.6);

        // Record the minimum cost of the shortest trajectory to the goal
        // Assume no change for joints config
        HashMap<Point2D, Double> verticesDis = new HashMap<>();
        verticesDis.put(endPos, 0.0);
        verticesDis.put(p1Pos, 0.424);
        verticesDis.put(p2Pos, 0.316);
        verticesDis.put(p3Pos, 0.341);
        verticesDis.put(p4Pos, 0.141);
        verticesDis.put(p5Pos, 0.588);
        verticesDis.put(p6Pos, 0.541);
        verticesDis.put(p7Pos, 0.141);
        verticesDis.put(p8Pos, 0.316);


        // Find the nearest path in visibility graph from the starting node 
        double shortestDist = Double.POSITIVE_INFINITY;
        //Point2D nearestVertex;
        for (Point2D p : verticesDis.keySet()) {
            double distFromVertex = startPos.distance(p) + verticesDis.get(p);
            // Update minimum cost if necessary
            if (distFromVertex < shortestDist) { shortestDist = distFromVertex; }           
        }
        
        return shortestDist;
    }


     /**
     * Calculate the probability of reaching the goal state given a trajectory
     * from start node to a end node
     * 
     * @param startNode
     *      the start node of the trajectory
     * @param endNode
     *      the end node of the trajectory
     * @param goalNode
     *      the goal node
     * 
     * @return
     *      probability of reaching the goal given this trajectory
     */
    public static double calcLegibleProbability(Node startNode, Node endNode, Node goalNode) {
        ArmConfig startConfig = startNode.getConfig();
        ArmConfig endConfig = endNode.getConfig();
        ArmConfig goalConfig = goalNode.getConfig();

        // Get the cost between S-Q and Q-G
        // (--- use total distance for now. will be modified later ---)
        double costSQ = cost(startNode, endNode);
        double bestCostSG = bestCost(startNode, goalNode);
        double bestCostQG = bestCost(endNode, goalNode);
        // Apply the formula
        double legiProb = Math.exp(-costSQ - bestCostQG) / Math.exp(-bestCostSG);

        return legiProb;
    }


    /**
     * Greedy search on current graph with legibility of trajectory included as heuristic
     * 
     * @param explored
     *      visited vertices. Avoid inifinite loop
     * @param currentNode
     *      currently checked vertex
     * 
     * @return
     *      return the goal node if can be reached. Otherwise return null
     *      
     */
    public static int greedySearchByLeg(ArrayList<Integer> explored, int currentIdx) {
        // Keep a explored list to avoid loop
        explored.add(currentIdx);
        // Known that the index of the goal node is 1 on the graph
        // Return once reach the goal node
        Node currentNode = vertices.get(currentIdx);
        if (currentIdx == 1){
            return 1;
        }
        
        // Calculate legibility from the current node to each of its candidate descendent
        List<Tuple<Integer, Double>> legiProbList = new ArrayList<>();
        for (int i = 0; i < vertices.size(); i++) {
            if (!explored.contains(i)) {
                Node endNode = vertices.get(i);
                Node goalNode = vertices.get(1);
                double legiProb = calcLegibleProbability(currentNode, endNode, goalNode);
                // Create new index-probability pair for node's legibility
                Tuple<Integer, Double> tuple = new Tuple<>(i, legiProb);
                legiProbList.add(tuple);
            }
        }
        // Sort the descendent nodes by legible probability in descending order
        Collections.sort(legiProbList, (tuple1, tuple2) -> tuple2.getValue().compareTo(tuple1.getValue()));

        // Select the possibly most legible node (highest heuristic)
        for (Tuple<Integer, Double> tuple : legiProbList) {
            // Take the childnode with best legible probability
            int childIdx = tuple.getIdx();
            Node childNode = vertices.get(childIdx);

            if (currentNode.distance(childNode) <= 0.5
                && !makePath(currentIdx, childIdx)
                && childNode.distance(vertices.get(1)) <= currentNode.distance(vertices.get(1))) {
                    // Set the current node parent of the child node
                    childNode.setParent(currentIdx);
                    vertices.set(childIdx, childNode);
                    // Continue greedy search
                    int result = greedySearchByLeg(explored, childIdx);
                    if (result == 1) {
                        return result;
                    }
            }
        }
        
        return -1;
    }


    /**
     * Greedy search on current graph with legibility of trajectory included as heuristic
     * 
     * @param explored
     *      visited vertices. Avoid inifinite loop
     * @param currentNode
     *      currently checked vertex
     * 
     * @return
     *      return the goal node if can be reached. Otherwise return null
     *      
     */
    public static int greedySearchByOpt(ArrayList<Integer> explored, int currentIdx) {
        // Keep a explored list to avoid loop
        explored.add(currentIdx);
        // Known that the index of the goal node is 1 on the graph
        // Return once reach the goal node
        Node currentNode = vertices.get(currentIdx);
        if (currentIdx == 1){
            return 1;
        }
        
        // Calculate legibility from the current node to each of its candidate descendent
        List<Tuple<Integer, Double>> costList = new ArrayList<>();
        for (int i = 0; i < vertices.size(); i++) {
            if (!explored.contains(i)) {
                Node endNode = vertices.get(i);
                //Node goalNode = vertices.get(1);
                double cost = cost(currentNode, endNode);
                // Create new index-probability pair for node's legibility
                Tuple<Integer, Double> tuple = new Tuple<>(i, cost);
                costList.add(tuple);
            }
        }

        // Sort the descendent nodes by legible probability in descending order
        Collections.sort(costList, (tuple1, tuple2) -> tuple1.getValue().compareTo(tuple2.getValue()));

        // Select the possibly most legible node (highest heuristic)
        for (Tuple<Integer, Double> tuple : costList) {
            // Take the childnode with best legible probability
            int childIdx = tuple.getIdx();
            Node childNode = vertices.get(childIdx);

            if (currentNode.distance(childNode) <= 0.5
                && !makePath(currentIdx, childIdx)
                && childNode.distance(vertices.get(1)) <= currentNode.distance(vertices.get(1))) {
                    // Set the current node parent of the child node
                    childNode.setParent(currentIdx);
                    vertices.set(childIdx, childNode);
                    // Continue greedy search
                    int result = greedySearchByOpt(explored, childIdx);
                    if (result == 1) {
                        return result;
                    }
            }
        }
        
        return -1;
    }


    /**
     * Depth-First-Sesearch on current graph
     * 
     * @param explored
     *      visited vertices. Avoid inifinite loop
     * @param currentNode
     *      currently checked vertex
     * 
     * @return
     *      return the goal node if can be reached. Otherwise return null
     *      
     */
    public static int dfs(ArrayList<Integer> explored, int currentIdx) {
        // Known that the index of the goal node is 1 on the graph
        // Return the goal node once found
        explored.add(currentIdx);
        Node currentNode = vertices.get(currentIdx);
        if (currentIdx == 1){
            return 1;
        }
        for (int i = 0; i < vertices.size(); i++) {
            Node childNode = vertices.get(i);
            if (!explored.contains(i) 
                && currentNode.distance(childNode) <= 0.5
                && !makePath(currentIdx, i)
                && childNode.distance(vertices.get(1)) <= currentNode.distance(vertices.get(1))) {
                    childNode.setParent(currentIdx);
                    vertices.set(i, childNode);
                    int result = dfs(explored, i);
                    if (result == 1) {
                        return result;
                    }
            }
        }
        return -1;
    }


    /**
     * Trace for the path and output to a file
     * 
     * @param node
     *      the goal node
     * @param solFile
     *      output file name
     */
    public static double outputPath(int idx, String solFile) {
        // Construct path
        List<ArmConfig> path = new ArrayList<>();

        // Initialize legibility required info
        ArrayList<Double> probList = new ArrayList<>();
        ArrayList<Integer> stepList = new ArrayList<>();

        // Trace the path
        while (vertices.get(idx).getParent() != -1) {
            path.add(vertices.get(idx).getConfig());
            path.addAll(vertices.get(idx).getPath());
            
            // Record legibility required info
            //probList.add(calcLegibleProbability(vertices.get(vertices.get(idx).getParent()), vertices.get(idx), vertices.get(1)));
            probList.add(calcLegibleProbability(vertices.get(0), vertices.get(idx), vertices.get(1)));
            stepList.add(path.size());

            idx = vertices.get(idx).getParent();
        }
        path.add(vertices.get(0).getConfig());
        Collections.reverse(path);

        // Calcualte the legibility
        Collections.reverse(probList);
        Collections.reverse(stepList);
        // Adjust values in the list
        ArrayList<Integer> ftList = new ArrayList<>();
        int totalDuration = stepList.get(0);
        int stepDuration = 0;
        for (int i = 0; i < stepList.size(); i++) {
            if (i != stepList.size() - 1) {
                stepDuration += stepList.get(i) - stepList.get(i + 1);
                ftList.add(totalDuration - stepDuration);
            } else {
                stepDuration += stepList.get(i);
                ftList.add(totalDuration - stepDuration);
            }
        }
        
        // Calculate legibility by taking integration (as summation) of weighted legible probability
        double probSum = 0;
        double ftSum = 0;
        for (int i = 0; i < ftList.size(); i++) {
            probSum += probList.get(i) * ftList.get(i);
            ftSum += ftList.get(i);
        }
        double legibility = probSum / ftSum;

        // Output result
        /*
        File outputFile = new File(solFile);
        try {
            BufferedWriter bw = new BufferedWriter(new FileWriter(outputFile));
            bw.write(Integer.toString(path.size()-1));
            bw.newLine();
            for (ArmConfig a : path) {
                bw.write(a.toString());
                bw.newLine();
            }
            bw.close();
        } catch (IOException e){
            e.printStackTrace();
        }
        */
        // Return a legibility value
        return legibility;
    }
 


    public static void main(String[] args) {
        // Initialize experiment statistics
        int legWin = 0;
        int optWin = 0;
        double legScore = 0;
        double optScore = 0;
        double dfsScore = 0;

        double verticesNum = 0;

        int legFound = 0;
        int optFound = 0;
        int dfsFound = 0;

        for (int i = 0; i < 30; i++) {

            try {
            // Initialize solver by problem spcification and a pre-constructed prm graph
                initialize(args[0], args[3]);
                System.out.println("Finished loading! - " + (i + 1));
            } catch (IOException e) {
                System.out.println("IO Exception occured");
            }

            verticesNum += vertices.size();
            
            double legLegibility = 0;
            double optLegibility = 0;
            double dfsLegibility = 0;
            //** Use Legibility Greedy Search to search for path */
            
            // Check if the goal is reached by Legibility Greedy Search
            
            int result_leg = greedySearchByLeg(new ArrayList<>(), 0);
            // We know the idx of the goal is "1"
            if (result_leg == 1) {
                System.out.println("Found path by Legibility Greedy Search.");
                legLegibility = outputPath(result_leg, args[1]);
                legFound++;
                //System.out.println("Path Legibility: " + legLegibility);
            }
            else {
                //System.out.println("result: " + result_leg);
                System.out.println("Failed to find a valid path by Leigibility Greedy Search.");
            }
            

            //** Use Optimality Greedy Search to search for path */
            /*
            // Check if the goal is reached by Optimality Greedy Search
            int result_opt = greedySearchByOpt(new ArrayList<>(), 0);
            // We know the idx of the goal is "1"
            if (result_opt == 1) {
                System.out.println("Found path by Optimality Greedy Search.");
                optLegibility = outputPath(result_opt, args[2]);
                optFound++;
                //System.out.println("Path Legibility: " + optLegibility);
            }
            else {
                //System.out.println("result: " + result_opt);
                System.out.println("Failed to find a valid path by Optimality Greedy Search.");
            }
            */

            //** Use Depth First Search to search for path */
            
            // Check if the goal is reached by Depth First Search
            int result_dfs = dfs(new ArrayList<>(), 0);
            // We know the idx of the goal is "1"
            if (result_dfs == 1) {
                System.out.println("Found path by Depth First Search.");
                dfsLegibility = outputPath(result_dfs, args[2]);
                dfsFound++;
                //System.out.println("Path Legibility: " + optLegibility);
            }
            else {
                //System.out.println("result: " + result_opt);
                System.out.println("Failed to find a valid path by Depth First Search.");
            }
            
            // Compare and record the result
            /*
            if (legLegibility > optLegibility) {
                legWin++;
            }   else if (optLegibility > legLegibility) {
                optWin++;
            }
            */
            legScore += legLegibility;
            optScore += optLegibility;
            dfsScore += dfsLegibility;

        }

        // Output statistics
        System.out.println();
        System.out.println("Legible Greedy won: " + legWin + "  Average legibility:" + legScore / legFound);
        //System.out.println("Optimal Greedy won: " + optWin + "  Average legibility:" + optScore / optFound);
        System.out.println("DFS Average legibility:" + dfsScore / dfsFound);
        System.out.println("Graph size: " + verticesNum / 30);

        //--- TEST ---/       

    }
}
