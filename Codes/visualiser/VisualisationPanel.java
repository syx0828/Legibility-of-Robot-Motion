package visualiser;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.Stroke;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.geom.*;
import java.util.List;

import javax.swing.JComponent;
import javax.swing.Timer;

import problem.Obstacle;
import problem.ArmConfig;
import problem.ProblemSpec;

public class VisualisationPanel extends JComponent {
	/** UID, as required by Swing */
	private static final long serialVersionUID = -4286532773714402501L;

	private ProblemSpec problemSetup = new ProblemSpec();
	private Visualiser visualiser;

	private AffineTransform translation = AffineTransform.getTranslateInstance(
			0, -1);
	private AffineTransform transform = null;

	private ArmConfig currentState;
	private boolean animating = false;
	private boolean displayingSolution = false;
	private Timer animationTimer;
	private int framePeriod = 20; // 50 FPS
	private Integer frameNumber = null;
	private int maxFrameNumber;

	private int samplingPeriod = 100;

	public VisualisationPanel(Visualiser visualiser) {
		super();
		this.setBackground(Color.WHITE);
		this.setOpaque(true);
		this.visualiser = visualiser;
	}

	public void setDisplayingSolution(boolean displayingSolution) {
		this.displayingSolution = displayingSolution;
		repaint();
	}

	public boolean isDisplayingSolution() {
		return displayingSolution;
	}

	public void setFramerate(int framerate) {
		this.framePeriod = 1000 / framerate;
		if (animationTimer != null) {
			animationTimer.setDelay(framePeriod);
		}
	}

	public void initAnimation() {
		if (!problemSetup.solutionLoaded()) {
			return;
		}
		if (animationTimer != null) {
			animationTimer.stop();
		}
		animating = true;
		gotoFrame(0);
		maxFrameNumber = problemSetup.getPath().size() - 1;
		animationTimer = new Timer(framePeriod, new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent arg0) {
				int newFrameNumber = frameNumber + 1;
				if (newFrameNumber >= maxFrameNumber) {
					animationTimer.stop();
					visualiser.setPlaying(false);
				}
				if (newFrameNumber <= maxFrameNumber) {
					gotoFrame(newFrameNumber);
				}
			}
		});
		visualiser.setPlaying(false);
		visualiser.updateMaximum();
	}

	public void gotoFrame(int frameNumber) {
		if (!animating
				|| (this.frameNumber != null && this.frameNumber == frameNumber)) {
			return;
		}
		this.frameNumber = frameNumber;
		visualiser.setFrameNumber(frameNumber);
		currentState = problemSetup.getPath().get(frameNumber);
		repaint();
	}

	public int getFrameNumber() {
		return frameNumber;
	}

	public void playPauseAnimation() {
		if (animationTimer.isRunning()) {
			animationTimer.stop();
			visualiser.setPlaying(false);
		} else {
			if (frameNumber >= maxFrameNumber) {
				gotoFrame(0);
			}
			animationTimer.start();
			visualiser.setPlaying(true);
		}
	}

	public void stopAnimation() {
		if (animationTimer != null) {
			animationTimer.stop();
		}
		animating = false;
		visualiser.setPlaying(false);
		frameNumber = null;
	}

	public ProblemSpec getProblemSetup() {
		return problemSetup;
	}

	public void calculateTransform() {
		transform = AffineTransform.getScaleInstance(getWidth(), -getHeight());
		transform.concatenate(translation);
	}

	public void paintState(Graphics2D g2, ArmConfig s) {
		if (s == null) {
			return;
		}
		Path2D.Float path = new Path2D.Float();

		List<Line2D> links = s.getLinks();
		Point2D p = s.getBaseCenter();
        path.moveTo(p.getX(), p.getY());

        // draw arm links
        if(s.hasGripper()) {
            for(int i = 0; i < (links.size() - 4); i++) {
                p = links.get(i).getP2();
                path.lineTo(p.getX(), p.getY());
            }

            // upper gripper
            Point2D p2 = links.get(links.size() - 4).getP2();
            path.lineTo(p2.getX(), p2.getY());
            Point2D p3 = links.get(links.size() - 3).getP2();
            path.lineTo(p3.getX(), p3.getY());

            path.moveTo(p.getX(), p.getY());

            // lower gripper
            p2 = links.get(links.size() - 2).getP2();
            path.lineTo(p2.getX(), p2.getY());
            p3 = links.get(links.size() - 1).getP2();
            path.lineTo(p3.getX(), p3.getY());

            path.transform(transform);
            g2.draw(path);

        } else {
            for(int i = 0; i < links.size(); i++) {
                p = links.get(i).getP2();
                path.lineTo(p.getX(), p.getY());
            }

            path.transform(transform);
            g2.draw(path);
        }

		if (animating || !displayingSolution) {
			p = transform.transform(s.getBaseCenter(), null);
			Color color = g2.getColor();
			Stroke stroke = g2.getStroke();
			g2.setColor(Color.BLACK);
			g2.setStroke(new BasicStroke(1));

            // draw chair base
            p = s.getBaseCenter();

            double x1 = (p.getX() - 0.02) * transform.getScaleX();
            double y1 = (0.98 - p.getY()) * transform.getScaleY() * -1;
            double w = 0.04 * transform.getScaleX();
            double h = 0.04 * transform.getScaleY() * -1;

            Rectangle2D.Double r = new Rectangle2D.Double(x1, y1, w, h);
            g2.draw(r);

			g2.setColor(color);
			g2.setStroke(stroke);
		}
	}

	public void setSamplingPeriod(int samplingPeriod) {
		this.samplingPeriod = samplingPeriod;
		repaint();
	}

	public void paintComponent(Graphics graphics) {
		super.paintComponent(graphics);
		if (!problemSetup.problemLoaded()) {
			return;
		}
		calculateTransform();
		Graphics2D g2 = (Graphics2D) graphics;
		g2.setColor(Color.WHITE);
		g2.fillRect(0, 0, getWidth(), getHeight());

		List<Obstacle> obstacles = problemSetup.getObstacles();
		if (obstacles != null) {
			g2.setColor(Color.red);
			for (Obstacle obs : problemSetup.getObstacles()) {
				Shape transformed = transform.createTransformedShape(obs
						.getRect());
				g2.fill(transformed);
			}
		}

		g2.setStroke(new BasicStroke(2));
		if (!animating) {
			if (displayingSolution) {
				List<ArmConfig> path = problemSetup.getPath();
				int lastIndex = path.size() - 1;
				for (int i = 0; i < lastIndex; i += samplingPeriod) {
					float t = (float) i / lastIndex;
					g2.setColor(new Color(0, t, 1 - t));
					paintState(g2, path.get(i));
				}
				g2.setColor(Color.green);
				paintState(g2, path.get(lastIndex));
			} else {
				g2.setColor(Color.blue);
				paintState(g2, problemSetup.getInitialState());

				g2.setColor(Color.green);
				paintState(g2, problemSetup.getGoalState());
			}
		} else {
			g2.setColor(Color.blue);
			paintState(g2, currentState);
		}
	}
}
