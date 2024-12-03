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
    private boolean targetReached = false;
    private double targetGlobalX = 0;
    private double targetGlobalY = 0;
    private boolean movingToSharedTarget = false;
    private static final double TARGET_REACHED_THRESHOLD = 0.15; 

	public AutonomyThree() {
		timeStep = 128;  
		random = new Random();

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
	 * Switch on / off a LED according to its num ([0;9])
	 * @param num
	 * @param on : true if the LED is to be switched on, 
	 * or false if the LED is to be switched off
	 */
	protected void setLED(int num, boolean on) {
		if(num < 10) {
			leds[num].set(on ? 1 : 0);
		}
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
	 * @param robot another robot detected by the Camera
	 * @return true if this robot has his LED "led8" on, false otherwise
	 */
	private boolean isLightON(CameraRecognitionObject robot) {
		int[] image=camera.getImage();
		boolean detected=false;

		int[] position=robot.getPositionOnImage();
		int width=robot.getSizeOnImage()[0];
		int height=robot.getSizeOnImage()[1];

		int startx=position[0] - (width + 1)/2;
		int starty=position[1] - (height + 1)/2;

		for (int i = 0; i < width; i++) {
			for(int j=0;j< height;j++) {
				int pixel=image[(startx+i)+(camera.getWidth() * (starty+j))];
				if(Camera.pixelGetRed(pixel) >= 254 && 	Camera.pixelGetGreen(pixel) >= 254 && Camera.pixelGetBlue(pixel) < 200) {
					if (detected) return true;
					else detected=true;					
				}
			}
		}
		return false;

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
	private boolean handleTargetDetection(List<CameraRecognitionObject> detectedObjects) {
        if (movingToSharedTarget) {
            return moveToSharedTarget();
        }

        CameraRecognitionObject target = targetDetected(detectedObjects);
        if (target != null) {
            // Positions locales par rapport au robot
            double[] localPosition = target.getPosition();
            double localX = localPosition[0]; 
            double localY = localPosition[1];

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


    public void run() {
        initLocalisation(); 
     
        while (step(timeStep) != -1) {
            double leftCurrent = leftMotorSensor.getValue();
            double rightCurrent = rightMotorSensor.getValue();
            
            odometry.track_step_pos(leftCurrent, rightCurrent);
            
            double[] psValues = readDistanceSensorValues();
            List<CameraRecognitionObject> detectedObjects = cameraDetection();

            String receivedMessage = checkMailBox();
            if (receivedMessage != null) {
                System.out.println("Received message: " + receivedMessage);
				movingToSharedTarget = true;  
            }
            if (handleTargetDetection(detectedObjects)) continue;

            if (handleObstacleAvoidance(psValues)) continue;

            if (!movingToSharedTarget) {
                moveRandomly();
            }
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


    private boolean moveToSharedTarget() {
        // Position actuelle du robot
        double robotX = odometry.getX();
        double robotY = odometry.getY();
        double robotTheta = Math.toRadians(odometry.getTheta());

        // Vecteur vers la cible en coordonnées globales
        double deltaX = (targetGlobalX - robotX)*0.01;
        double deltaY = (targetGlobalY - robotY)*0.01;

        // Conversion en coordonnées locales pour la navigation
        double localX = deltaX * Math.cos(-robotTheta) - deltaY * Math.sin(-robotTheta);
        double localY = deltaX * Math.sin(-robotTheta) + deltaY * Math.cos(-robotTheta);

        // Distance à la cible
        double distanceToTarget = Math.sqrt(deltaX * localX + deltaY * localY);
        
        System.out.println("Moving to target - Distance: " + distanceToTarget);
        System.out.println("Robot at: X=" + robotX + ", Y=" + robotY);
        System.out.println("Target at: X=" + targetGlobalX + ", Y=" + targetGlobalY);
        System.out.println("Local vector to target: X=" + localX + ", Y=" + localY);

        if (distanceToTarget < TARGET_REACHED_THRESHOLD) {
            movingToSharedTarget = false;
            targetReached = true;
            System.out.println("Reached shared target!");
            move(0.0, 0.0);
            return false;
        }

        // Calculer l'angle dans le repère local
        double angleToTarget = Math.atan2(localY, localX);
        System.out.println("Angle to target: " + Math.toDegrees(angleToTarget));

        // Navigation
        if (Math.abs(angleToTarget) > Math.PI/6) {
            if (angleToTarget > 0) {
                move(15.0, -15.0);
            } else {
                move(-15.0, 15.0);
            }
        } else {
            double baseSpeed = 20.0;
            double correction = (angleToTarget / (Math.PI/6)) * 10.0;
            move(baseSpeed - correction, baseSpeed + correction);
        }

        return true;
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