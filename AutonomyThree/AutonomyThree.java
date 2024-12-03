import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.CameraRecognitionObject;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Emitter;
import com.cyberbotics.webots.controller.LED;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.PositionSensor;
import com.cyberbotics.webots.controller.Receiver;
import com.cyberbotics.webots.controller.Robot;
import java.util.Locale;  


public class AutonomyThree extends Robot {
	private int timeStep;
	private DistanceSensor[] distanceSensor;	
	private Motor leftMotor;
	private Motor rightMotor;
	private PositionSensor leftMotorSensor;
	private PositionSensor rightMotorSensor;
	private double encoder_unit=159.23;
	private Odometry odometry = new Odometry();	
	private Camera camera;
	private Emitter emitter;
	private Receiver receiver;
	private LED[] leds;
	private Random random;
	// Nouveaux attributs pour la coordination
    private boolean targetReached = false;
    private double targetGlobalX = 0;
    private double targetGlobalY = 0;
    private boolean movingToSharedTarget = false;
    // Ajuster le seuil de la cible atteinte
    private static final double TARGET_REACHED_THRESHOLD = 0.16; // Augmenté pour plus de tolérance
    private static final double MOVEMENT_SPEED_FACTOR = 1.5;    // Facteur pour ajuster la vitesse générale
	private boolean inAssignedZone = false;
private int zoneNumber;  // numéro de la zone (0-5)

	public AutonomyThree() {
		timeStep = 128;  // set the control time step
		random = new Random();
	    zoneNumber = random.nextInt(6);
        assignZone();
		// Sensors initialization 
		// IR distance sensors
		distanceSensor = new DistanceSensor[8];
		String[] sensorNames = {
				"ps0", "ps1", "ps2", "ps3",
				"ps4", "ps5", "ps6", "ps7"
		};

		for (int i = 0; i < 8; i++) {
			distanceSensor[i] = this.getDistanceSensor(sensorNames[i]);
			distanceSensor[i].enable(timeStep);
		}

		// Camera
		camera=this.getCamera("camera");
		camera.enable(timeStep);
		camera.recognitionEnable(timeStep);

		//  WiFi communication
		emitter=getEmitter("emitter");
		receiver=getReceiver("receiver");
		receiver.enable(timeStep);

		// Actuators initialization
		// Motors
		leftMotor = this.getMotor("left wheel motor");
		rightMotor = this.getMotor("right wheel motor");
		leftMotor.setPosition(Double.POSITIVE_INFINITY);
		rightMotor.setPosition(Double.POSITIVE_INFINITY);
		leftMotor.setVelocity(0.0);
		rightMotor.setVelocity(0.0);

		// Motor sensors : to compute relative position
		leftMotorSensor = this.getPositionSensor("left wheel sensor");
		rightMotorSensor = this.getPositionSensor("right wheel sensor");
		leftMotorSensor.enable(timeStep);
		rightMotorSensor.enable(timeStep);

		// LEDS
		leds = new LED[10];
		String[] ledsNames = {
				"led0", "led1", "led2", "led3",
				"led4", "led5", "led6", "led7",
				"led8", "led9"
		};
		for (int i = 0; i < 10; i++) {
			leds[i] = this.getLED(ledsNames[i]);
		}
	}
	
	

    private boolean handleObstacleAvoidance(double[] psValues) {
        double threshold = 100.0;
        int aleatoire = random.nextInt(-20, 25);

        if ((psValues[0] < threshold && psValues[7] < threshold)) {
            move(50 + aleatoire, 50 - aleatoire);
            return true;
        }

        if ((psValues[2] > threshold) ||
            (psValues[0] > threshold && psValues[1] > threshold && psValues[2] > threshold) ||
            (psValues[0] > threshold && psValues[1] > threshold && psValues[7] > threshold) ||
            (psValues[0] > threshold && psValues[1] > threshold) ||
            (psValues[0] > threshold) ||
            (psValues[1] > threshold) ||
            (psValues[1] > threshold && psValues[2] > threshold)) {
            move(-50.0, 50.0);
            return true;
        }

        if ((psValues[0] > threshold && psValues[7] > threshold) ||
            (psValues[0] > threshold && psValues[7] > threshold && psValues[1] > threshold && psValues[6] > threshold) ||
            (psValues[1] > threshold && psValues[7] > threshold && psValues[6] > threshold) ||
            (psValues[0] > threshold && psValues[1] > threshold && psValues[6] > threshold)) {
            int direction = random.nextInt(2);
            if (direction == 1) move(-50.0, 50.0);
            else move(50.0, -50.0);
            return true;
        }

        if ((psValues[5] > threshold) ||
            (psValues[7] > threshold) ||
            (psValues[5] > threshold && psValues[6] > threshold) ||
            (psValues[6] > threshold && psValues[7] > threshold) ||
            (psValues[5] > threshold && psValues[6] > threshold && psValues[7] > threshold) ||
            (psValues[0] > threshold && psValues[6] > threshold && psValues[7] > threshold)) {
            move(50.0, -50.0);
            return true;
        }

        return false;
    }
	/**
	 * 
	 * @param left : a value between [-100;100]%
	 * @param right : a value between [-100;100]%
	 */
	protected void move(double left, double right) {
		double max=6.2;
		getMotor("left wheel motor").setVelocity(left * max / 100);
		getMotor("right wheel motor").setVelocity(right * max / 100);
	}

	/**
	 * 
	 * @return an empty list if nothing is detected by the camera, 
	 * a list of CameraRecognitionObject otherwise (see https://cyberbotics.com/doc/reference/camera#camera-recognition-object)
	 */
	protected List<CameraRecognitionObject> cameraDetection() {
		ArrayList<CameraRecognitionObject> detected=new ArrayList<>();
		int nb=camera.getRecognitionNumberOfObjects();
		if(nb >0) {
			CameraRecognitionObject[] objects=camera.getRecognitionObjects();
			for(int i=0;i<objects.length;i++) {
				detected.add(objects[i]);
			}
		}
		return detected;
	}
	/**
	 * Look in a List of camera detected objects if the target is one of them 
	 * @param detected: a List of camera detected objects
	 * @return the target (a specific CameraRecognitionObject) or null
	 */
	protected CameraRecognitionObject targetDetected(List<CameraRecognitionObject> detected) {
		for(CameraRecognitionObject ob:detected) {
			if(ob.getModel().compareTo("cible") == 0)
				return ob;
		}
		return null;		
	}

	/**
	 * 
	 * @return a double array with values for each IR sensor 
	 * Each value is between approx. [67 ; 750 (very close - contact)]
	 * (see https://cyberbotics.com/doc/guide/epuck)
	 */
	protected double[] readDistanceSensorValues() {
		// read sensors outputs
		double[] psValues = {0, 0, 0, 0, 0, 0, 0, 0};
		for (int i = 0; i < 8 ; i++)
			psValues[i] = distanceSensor[i].getValue();

		return psValues;
	}
	/**
	 * Initialisation of the computation of the relative position of the robot
	*/
	private void initLocalisation() {		
		step(timeStep);
		odometry.track_start_pos(encoder_unit * leftMotorSensor.getValue(), encoder_unit * rightMotorSensor.getValue());
		odometry.setX(0);
		odometry.setY(0);
		odometry.setTheta(Math.PI/2);		
	}
	
	/**
	 * To call to compute in real time its own relative position 
	 * @param print : true if the relative position must be printed in the console 
	 */
	protected void localise(boolean print) {
		odometry.track_step_pos(encoder_unit * leftMotorSensor.getValue(), encoder_unit * rightMotorSensor.getValue());
		if(print) {
			Double[] pos=getPosition();
			System.out.println("Position : "+pos[0]+","+pos[1]);
		}
	}
	/**
	 * Get the computed relative position of the robot (x;y)
	 * The starting point is always (0;0)
	 * @return
	 */
	protected Double[] getPosition() {
		return new Double[] {odometry.getX(),odometry.getY()};
	}
//added code ********************
	private boolean handleTargetDetection(List<CameraRecognitionObject> detectedObjects) {
        CameraRecognitionObject target = targetDetected(detectedObjects);
        if (target != null) {
            // Positions locales par rapport au robot
            double[] localPosition = target.getPosition();
            double localX = localPosition[0]; // avant/arrière
            double localY = localPosition[1]; // gauche/droite

            // Position et orientation actuelles du robot
            double robotX = odometry.getX();
            double robotY = odometry.getY();
            double robotTheta = Math.toRadians(odometry.getTheta());

            // Conversion en coordonnées globales
            double targetX_global = robotX + (localX * Math.cos(robotTheta) - localY * Math.sin(robotTheta));
            double targetY_global = robotY + (localX * Math.sin(robotTheta) + localY * Math.cos(robotTheta));

            // Distance locale pour la détection
            double distanceToTarget = Math.sqrt(localX * localX + localY * localY);
            
            if (distanceToTarget < TARGET_REACHED_THRESHOLD && !targetReached) {
                targetReached = true;
                
                String targetReachedMessage = String.format(Locale.US, "TARGET_REACHED:%.6f:%.6f", 
                    targetX_global, targetY_global);
                System.out.println("Found target! Broadcasting position: " + targetReachedMessage);
                broadcastMessage(targetReachedMessage);
                
                for (LED led : leds) {
                    led.set(1);
                }
                move(0.0, 0.0);
                return true;
            }

            // Navigation en utilisant les coordonnées locales
            double angleToTarget = Math.atan2(localY, localX);
            if (Math.abs(angleToTarget) > 0.1) {
                move(-5.0, 25.0);
            } else {
                move(25.0, 25.0);
            }
            return true;
        }
        return false;
    }


private static final int TOTAL_ROBOTS = 6;
private int robotId; // Identifiant unique du robot (0-5)
private double zoneMinX, zoneMaxX, zoneMinY, zoneMaxY; // Limites de la zone assignée
private static final double ARENA_SIZE_X = 2.0; // Taille totale de l'arène en X
private static final double ARENA_SIZE_Y = 3.0; // Taille totale de l'arène en Y






// Modifier la méthode run()
public void run() {

    initLocalisation();

    while (step(timeStep) != -1) {
        localise(false);
        Double[] currentPos = getPosition();
        double[] psValues = readDistanceSensorValues();
        List<CameraRecognitionObject> detectedObjects = cameraDetection();

        String receivedMessage = checkMailBox();
        
        if (movingToSharedTarget) {
            System.out.println("Déplacement vers la cible partagée");
            if (!handleObstacleAvoidance(psValues)) {
                navigateToSharedTarget();
            }
        } else if (handleTargetDetection(detectedObjects)) {
            continue;
        } else if (handleObstacleAvoidance(psValues)) {
            continue;
        } else if (!inAssignedZone) {
            // Se diriger vers la zone assignée
            moveToAssignedZone();
        } else {
            // Explorer la zone assignée
            exploreZone(psValues);
        }
    }
}

private void assignZone() {
    // Calculer la position de la zone basée sur zoneNumber
    int row = zoneNumber / 3;    // 0 pour zones 0,1,2 et 1 pour zones 3,4,5
    int col = zoneNumber % 3;    // 0,1,2 pour chaque rangée
    
    double zoneWidth = ARENA_SIZE_X / 3;
    double zoneHeight = ARENA_SIZE_Y / 2;
    //pour encadrer chaque zone entre (zoneMinX,zoneMinY),(zoneMaxX,zoneMaxXy)
    zoneMinX = col * zoneWidth - ARENA_SIZE_X/2;
    zoneMaxX = (col + 1) * zoneWidth - ARENA_SIZE_X/2;
    zoneMinY = row * zoneHeight - ARENA_SIZE_Y/2;
    zoneMaxY = (row + 1) * zoneHeight - ARENA_SIZE_Y/2;
    
    System.out.println("Robot assigned to zone " + zoneNumber + ": (" + 
        zoneMinX + "," + zoneMinY + ") to (" + zoneMaxX + "," + zoneMaxY + ")");
}

private void moveToAssignedZone() {
    Double[] currentPos = getPosition();
    double targetX = (zoneMinX + zoneMaxX) / 2;
    double targetY = (zoneMinY + zoneMaxY) / 2;
    
    if (currentPos[0] >= zoneMinX && currentPos[0] <= zoneMaxX &&
        currentPos[1] >= zoneMinY && currentPos[1] <= zoneMaxY) {
        inAssignedZone = true;
        return;
    }
    
    double deltaX = targetX - currentPos[0];
    double deltaY = targetY - currentPos[1];
    double angle = Math.atan2(deltaY, deltaX);
    double robotTheta = odometry.getTheta();
    double angleDiff = angle - robotTheta;
    
    while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
    while (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;
    
    if (Math.abs(angleDiff) > 0.1) {
        move(angleDiff > 0 ? -15.0 : 15.0, angleDiff > 0 ? 15.0 : -15.0);
    } else {
        move(30.0, 30.0);
    }
}

private void exploreZone(double[] psValues) {
    Double[] currentPos = getPosition();
    
    if (currentPos[0] < zoneMinX || currentPos[0] > zoneMaxX ||
        currentPos[1] < zoneMinY || currentPos[1] > zoneMaxY) {
        inAssignedZone = false;
        return;
    }
    
    double distanceFromMinX = currentPos[0] - zoneMinX;
    double distanceFromMaxX = zoneMaxX - currentPos[0];
    
    if (distanceFromMinX < 0.1 || distanceFromMaxX < 0.1) {
        // Près des bords, changer de direction
        if (distanceFromMinX < 0.1) {
            move(20.0, -20.0);  // tourner droite
        } else {
            move(-20.0, 20.0);  // tourner gauche
        }
    } else {
        // Exploration avec légère oscillation
        double oscillation = Math.sin(System.currentTimeMillis() * 0.001) * 5.0;
        move(25.0 + oscillation, 25.0 - oscillation);
    }
}

// Modifier la méthode navigateToSharedTarget pour une meilleure navigation
private void navigateToSharedTarget() {
    Double[] currentPos = getPosition();
    double deltaX = targetGlobalX - currentPos[0];
    double deltaY = targetGlobalY - currentPos[1];
    double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    
    if (distance < TARGET_REACHED_THRESHOLD) {
        targetReached = true;
        move(0.0, 0.0);
        return;
    }
    
    double angleToTarget = Math.atan2(deltaY, deltaX);
    double robotTheta = odometry.getTheta();
    double angleDiff = angleToTarget - robotTheta;
    
    while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
    while (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;
    
    if (Math.abs(angleDiff) > 0.1) {
        move(angleDiff > 0 ? -15.0 : 15.0, angleDiff > 0 ? 15.0 : -15.0);
    } else {
        move(30.0, 30.0);
    }
}
	/**
	 * Allows to send a message at all the other robots
	 * @param message
	 */
	protected void broadcastMessage(String message) {
		emitter.send(message.getBytes());
	}
		/**
	 * Check if a message has been received, and flush the pile
	 * @return null if there is no message, a String otherwise
	 */
	protected String checkMailBox() {
        String message = null;
        while (receiver.getQueueLength() > 0) {
            byte[] data = receiver.getData();
            receiver.nextPacket();
            
            if (data != null) {
                message = new String(data);
                System.out.println("Received raw message: " + message);
                
                if (message.startsWith("TARGET_REACHED:")) {
                    String[] parts = message.split(":");
                    if (parts.length == 3) {
                        try {
                            double x = Double.parseDouble(parts[1].replace(',', '.'));
                            double y = Double.parseDouble(parts[2].replace(',', '.'));
                            
                            // Debug values before assignment
                            System.out.println("Parsed coordinates - X: " + x + ", Y: " + y);
                            
                            // Assigner les valeurs
                            targetGlobalX = x;
                            targetGlobalY = y;
                            movingToSharedTarget = true;
                            
                            System.out.println("Successfully set target position to: (" + targetGlobalX + ", " + targetGlobalY + ")");
                        } catch (Exception e) {
                            System.out.println("Failed to parse coordinates from: " + message);
                            e.printStackTrace();
                        }
                    }
                }
            }
        }
        return message;
    }

    private void moveRandomly() {
        double leftSpeed = random.nextDouble() * 100 - 50;
        double rightSpeed = random.nextDouble() * 100 - 50;
        move(leftSpeed, rightSpeed);
    }



	public static void main(String[] args) {
		AutonomyThree controller = new AutonomyThree();
		controller.run();
	}


	/**
	 * Do NOT modify
	 * Private class providing tools to compute a relative position for the robot
	 */
	private class Odometry{
		private double wheel_distance;
		private double wheel_conversion_left;
		private double wheel_conversion_right;
		private double pos_left_prev;
		private double pos_right_prev;
		private double x;
		private double y;
		private double theta;

		private double increments_per_tour = 1000.0;   // from e-puck.org
		private double axis_wheel_ratio = 1.4134;      // from e-puck.org
		private double wheel_diameter_left = 0.0416;   // from e-puck.org
		private double wheel_diameter_right = 0.0416;  // from e-puck.org
		private double scaling_factor = 0.976;         // default is 1

		public Odometry() {
			// TODO Auto-generated constructor stub
		}

		public int track_start_pos(double pos_left, double pos_right) {
			x=0;
			y=0;
			theta =0;

			pos_left_prev=pos_left;
			pos_right_prev=pos_right;

			wheel_distance = axis_wheel_ratio * scaling_factor * (wheel_diameter_left + wheel_diameter_right) / 2;
			wheel_conversion_left = wheel_diameter_left * scaling_factor * Math.PI / increments_per_tour;
			wheel_conversion_right = wheel_diameter_right * scaling_factor * Math.PI / increments_per_tour;

			return 1;
		}

		public void track_step_pos(double pos_left, double pos_right) {
			double delta_pos_left, delta_pos_right;
			double delta_left, delta_right, delta_theta, theta2;
			double delta_x, delta_y;

			delta_pos_left = pos_left - pos_left_prev;
			delta_pos_right = pos_right - pos_right_prev;
			delta_left = delta_pos_left * wheel_conversion_left;
			delta_right = delta_pos_right * wheel_conversion_right;
			delta_theta = (delta_right - delta_left) / wheel_distance;
			theta2 = theta + delta_theta * 0.5;
			delta_x = (delta_left + delta_right) * 0.5 * Math.cos(theta2);
			delta_y = (delta_left + delta_right) * 0.5 * Math.sin(theta2);

			x += delta_x;
			y += delta_y;
			theta += delta_theta;

			if(theta < 0)
				theta +=2 * Math.PI;
			if(theta > 2 * Math.PI)
				theta -=2 * Math.PI;

			pos_left_prev = pos_left;
			pos_right_prev = pos_right;
		}

		public double getX() {
			return x;
		}

		public void setX(double x) {
			this.x = x;
		}

		public double getY() {
			return y;
		}

		public void setY(double y) {
			this.y = y;
		}

		public double getTheta() {
			return theta;
		}

		public void setTheta(double theta) {
			this.theta = theta;
		}
	}
}