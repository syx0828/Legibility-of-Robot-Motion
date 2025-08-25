package problem;


import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.InputMismatchException;
import java.util.List;
import java.util.Scanner;

/** 
 * Represents a configuration of the arm, i.e. base x-y coordinates and joint
 * angles. This class doesn't do any validity checking.
 */
public class ArmConfig {
	
	/** Length of each link */
	public static final double LINK_LENGTH = 0.05;
    /** Width of chair */
    public static final double CHAIR_WIDTH = 0.04;

	/** Base x and y coordinates */
	private Point2D base;
    /** Boundary of chair as Line2D */
    private List<Line2D> chair;
	/** Joint angles in radians */
	private List<Double> jointAngles;
	/** Links as Line2D */
	private List<Line2D> links;

	/** Whether this configuration has a gripper */
	private boolean gripper;
	/** Lengths of gripper segments */
	private List<Double> gripperLengths;

	
	/**
	 * Constructor (no gripper)
	 * @param base
	 * 			Base coordinates (Point2D)
	 * @param jointAngles
	 * 			Joint angles in radians
	 */
	public ArmConfig(Point2D base, List<Double> jointAngles) {
		this.base = new Point2D.Double(base.getX(), base.getY());
		this.jointAngles = new ArrayList<Double>(jointAngles);
		this.gripper = false;
		generateLinks();
		generateChair();
	}

	/**
	 * Constructor (with gripper)
	 * @param base
	 * 			Base coordinates (Point2D)
	 * @param jointAngles
	 * 			Joint angles in radians
	 */
	public ArmConfig(Point2D base, List<Double> jointAngles, List<Double> gripperLengths) {
		this.base = new Point2D.Double(base.getX(), base.getY());
		this.jointAngles = new ArrayList<Double>(jointAngles);
		this.gripperLengths = new ArrayList<Double>(gripperLengths);
		this.gripper = true;
		generateLinks();
        	generateChair();
	}
	
	/** 
	 * Constructs an ArmConfig from a space-separated string. The first two
	 * numbers are the x and y coordinates of the robot's base. The subsequent
	 * numbers are the joint angles in sequential order defined in radians.
	 * 
	 * @param str
	 * 			The String containing the values
	 * 
	 * @throws InputMismatchException
	 */
	public ArmConfig(String str) throws InputMismatchException {
		Scanner s = new Scanner(str);
		base = new Point2D.Double(s.nextDouble(), s.nextDouble());
		jointAngles = new ArrayList<Double>();
		while (s.hasNextDouble()) {
			jointAngles.add(s.nextDouble());
		}
		s.close();
		this.gripper = false;
		generateLinks();
        	generateChair();
	}

	/**
	 * Constructs an ArmConfig from a space-separated string. The first two
	 * numbers are the x and y coordinates of the robot's base. The subsequent
	 * numbers are the joint angles in sequential order defined in radians.
	 *
	 * @param str
	 * 			The String containing the values
	 *
	 * @throws InputMismatchException
	 */
	public ArmConfig(String str, boolean gripper) throws InputMismatchException {

		this.gripper = gripper;

		Scanner s = new Scanner(str);
		base = new Point2D.Double(s.nextDouble(), s.nextDouble());
		jointAngles = new ArrayList<Double>();
		gripperLengths = new ArrayList<Double>();

		List<Double> tokens = new ArrayList<Double>();
		while (s.hasNextDouble()) {
			tokens.add(s.nextDouble());
		}

		if(this.gripper) {
			jointAngles.addAll(tokens.subList(0, tokens.size() - 4));
			gripperLengths.addAll(tokens.subList(tokens.size() - 4, tokens.size()));
		} else {
			jointAngles = tokens;
		}

		s.close();
		generateLinks();
        generateChair();
	}
	
	/**
	 * Copy constructor.
	 *
	 * @param cfg
	 *            the configuration to copy.
	 */
	public ArmConfig(ArmConfig cfg) {
		base = cfg.getBaseCenter();
		jointAngles = cfg.getJointAngles();
		links = cfg.getLinks();
        chair = cfg.getChair();
	}
	
	/**
	 * Returns a space-separated string representation of this configuration.
	 *
	 * @return a space-separated string representation of this configuration.
	 */
	public String toString() {
		StringBuilder sb = new StringBuilder();
		sb.append(base.getX());
        sb.append(" ");
		sb.append(base.getY());
		for (Double angle : jointAngles) {
			sb.append(" ");
			sb.append(angle);
		}
		if (gripper) {
            for (Double gripper : gripperLengths) {
                sb.append(" ");
                sb.append(gripper);
            }
        }
		return sb.toString();
	}
	
	/**
	 * Returns the number of joints in this configuration.
	 * 
	 * @return the number of joints in this configuration.
	 */
	public int getJointCount() {
		return jointAngles.size();
	}
	
	/**
	 * Returns the base position.
	 * 
	 * @return the base position.
	 */
	public Point2D getBaseCenter() {
		return new Point2D.Double(base.getX(), base.getY());
	}
	
	/**
	 * Returns the list of joint angles in radians.
	 * 
	 * @return the list of joint angles in radians.
	 */
	public List<Double> getJointAngles() {
		return new ArrayList<Double>(jointAngles);
	}

    /**
     * Returns whether the arm has a gripper.
     *
     * @return whether the arm has a gripper.
     */
	public boolean hasGripper() { return gripper; }

	/**
	 * Returns the list of gripper lengths.
	 *
	 * @return the list of gripper lengths.
	 */
	public List<Double> getGripperLengths() { return new ArrayList<Double>(gripperLengths); }
	
	/**
	 * Returns the list of links as Line2D.
	 * 
	 * @return the list of links as Line2D.
	 */
	public List<Line2D> getLinks() {
		return new ArrayList<Line2D>(links);
	}

    /**
     * Returns the chair boundary as list of Line2D.
     *
     * @return the chair boundary as list of Line2D.
     */
	public List<Line2D> getChair() { return new ArrayList<Line2D>(chair); }
	
	/**
	 * Returns the maximum straight-line distance between the link endpoints
	 * in this state vs. the other state, or -1 if the link counts don't match.
	 * 
	 * @param otherState
	 *            the other state to compare.
	 * @return the maximum straight-line distance for any link endpoint.
	 */
	public double maxDistance(ArmConfig otherState) {
		if (this.getJointCount() != otherState.getJointCount()) {
			return -1;
		}
		double maxDistance = base.distance(otherState.getBaseCenter());
		List<Line2D> otherLinks = otherState.getLinks();
		for (int i = 0; i < links.size(); i++) {
			double distance = links.get(i).getP2().distance(
					otherLinks.get(i).getP2());
			if (distance > maxDistance) {
				maxDistance = distance;
			}
		}
		return maxDistance;
	}
	
	/**
	 * Returns the total straight-line distance between the link endpoints
	 * in this state vs. the other state, or -1 if the link counts don't match.
	 * 
	 * @param otherState
	 *            the other state to compare.
	 * @return the total straight-line distance over all link endpoints.
	 */
	public double totalDistance(ArmConfig otherState) {
		if (this.getJointCount() != otherState.getJointCount()) {
			return -1;
		}
		if (this.hasGripper() != otherState.hasGripper()) {
		    return -1;
        }
		double totalDist = base.distance(otherState.getBaseCenter());
		List<Line2D> otherLinks = otherState.getLinks();
		for (int i = 0; i < links.size(); i++) {
			totalDist += links.get(i).getP2().distance(
					otherLinks.get(i).getP2());
		}
		return totalDist;
	}

	
	/**
	 * Returns the maximum difference in angle between the joints in this state
	 * vs. the other state, or -1 if the joint counts don't match.
	 * 
	 * @param otherState
	 *            the other state to compare.
	 * @return the maximum joint angle change for any joint.
	 */
	public double maxAngleDiff(ArmConfig otherState) {
		if (this.getJointCount() != otherState.getJointCount()) {
			return -1;
		}
		List<Double> otherJointAngles = otherState.getJointAngles();
		double maxDiff = 0;
		for (int i = 0; i < jointAngles.size(); i++) {
			double diff = Math.abs(jointAngles.get(i) - otherJointAngles.get(i));
			if (diff > maxDiff) {
				maxDiff = diff;
			}
		}
		return maxDiff;
	}

	/**
	 * Returns the maximum difference in gripper lengths between this state
	 * vs. the other state, or 0 if the configuration has no gripper.
	 *
	 * @param otherState
	 *            the other state to compare.
	 * @return the maximum gripper length change for any joint.
	 */
	public double maxGripperDiff(ArmConfig otherState) {
		if(!gripper) {
			return 0.0;
		}

		List<Double> otherGripperLengths = otherState.getGripperLengths();
		double maxDiff = 0;
		for (int i = 0; i < gripperLengths.size(); i++) {
			double diff = Math.abs(gripperLengths.get(i) - otherGripperLengths.get(i));
			if (diff > maxDiff) {
				maxDiff = diff;
			}
		}
		return maxDiff;
	}
	
	/**
	 * Generates links from joint angles
	 */
	private void generateLinks() {
		links = new ArrayList<Line2D>();
		double x1 = base.getX();
		double y1 = base.getY();
		double totalAngle = 0;
		for (Double angle : jointAngles) {
			totalAngle += angle;
			double x2 = x1 + LINK_LENGTH * Math.cos(totalAngle);
			double y2 = y1 + LINK_LENGTH * Math.sin(totalAngle);
			links.add(new Line2D.Double(x1, y1, x2, y2));
			x1 = x2;
			y1 = y2;
		}

		if(gripper) {
            // add link for u1
            double x2 = (gripperLengths.get(0) * Math.cos(totalAngle + (Math.PI / 2))) + x1;
            double y2 = (gripperLengths.get(0) * Math.sin(totalAngle + (Math.PI / 2))) + y1;
            links.add(new Line2D.Double(x1, y1, x2, y2));

            // add link for u2
            double x3 = (gripperLengths.get(1) * Math.cos(totalAngle)) + x2;
            double y3 = (gripperLengths.get(1) * Math.sin(totalAngle)) + y2;
            links.add(new Line2D.Double(x2, y2, x3, y3));

            // add link for l1
            x2 = (gripperLengths.get(0) * Math.cos(totalAngle - (Math.PI / 2))) + x1;
            y2 = (gripperLengths.get(0) * Math.sin(totalAngle - (Math.PI / 2))) + y1;
            links.add(new Line2D.Double(x1, y1, x2, y2));

            // add link for l2
            x3 = (gripperLengths.get(1) * Math.cos(totalAngle)) + x2;
            y3 = gripperLengths.get(1) * Math.sin(totalAngle) + y2;
            links.add(new Line2D.Double(x2, y2, x3, y3));
        }
	}

	private void generateChair() {
	    chair = new ArrayList<Line2D>();
        double x1 = base.getX();
        double y1 = base.getY();
        double halfWidth = CHAIR_WIDTH / 2;

        // generate in clockwise order (for bounds check)
        chair.add(new Line2D.Double(x1 - halfWidth, y1 + halfWidth, x1 + halfWidth, y1 + halfWidth));
        chair.add(new Line2D.Double(x1 + halfWidth, y1 + halfWidth, x1 + halfWidth, y1 - halfWidth));
        chair.add(new Line2D.Double(x1 + halfWidth, y1 - halfWidth, x1 - halfWidth, y1 - halfWidth));
        chair.add(new Line2D.Double(x1 - halfWidth, y1 - halfWidth, x1 - halfWidth, y1 + halfWidth));
    }

}
