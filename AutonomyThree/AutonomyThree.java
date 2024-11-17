
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

public class AutonomyThree extends Robot {
    private int timeStep;
    private DistanceSensor[] distanceSensor;
    private Motor leftMotor;
    private Motor rightMotor;
    private PositionSensor leftMotorSensor;
    private PositionSensor rightMotorSensor;
    private double encoder_unit = 159.23;
    private Camera camera;
    private Emitter emitter;
    private Receiver receiver;
    private LED[] leds;
    private Random random;
    private Odometry odometry;
    
    public AutonomyThree() {
        random = new Random();
        timeStep = 128; // set the control time step
        odometry = new Odometry();

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
        camera = this.getCamera("camera");
        camera.enable(timeStep);
        camera.recognitionEnable(timeStep);

        // WiFi communication
        emitter = getEmitter("emitter");
        receiver = getReceiver("receiver");
        receiver.enable(timeStep);

        // Actuators initialization
        // Motors
        leftMotor = this.getMotor("left wheel motor");
        rightMotor = this.getMotor("right wheel motor");
        leftMotor.setPosition(Double.POSITIVE_INFINITY);
        rightMotor.setPosition(Double.POSITIVE_INFINITY);
        leftMotor.setVelocity(0.0);
        rightMotor.setVelocity(0.0);

        // Motor sensors: to compute relative position
        leftMotorSensor = this.getPositionSensor("left wheel sensor");
        rightMotorSensor = this.getPositionSensor("right wheel sensor");
        leftMotorSensor.enable(timeStep);
        rightMotorSensor.enable(timeStep);

        // LEDs
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

    /**
     * The main method of the robot behaviour
     */
    public void run() {
        while (step(timeStep) != -1) {
            double[] psValues = readDistanceSensorValues();
            List<CameraRecognitionObject> detectedObjects = cameraDetection();

            String receivedMessage = checkMailBox();
            if (receivedMessage != null) {
                System.out.println("Received message: " + receivedMessage);
            }

            if (handleTargetDetection(detectedObjects)) continue;

            if (handleObstacleAvoidance(psValues)) continue;

            moveRandomly();
        }
    }

    /**
     * Move randomly if no other tasks are pending.
     */
    private void moveRandomly() {
        double leftSpeed = random.nextDouble() * 100 - 50;
        double rightSpeed = random.nextDouble() * 100 - 50;
        move(leftSpeed, rightSpeed);
    }

    /**
     * Broadcast a message to other robots.
     */
    protected void broadcastMessage(String message) {
                    System.out.println("sendddd message: " + message);

        emitter.send(message.getBytes());
    }


    protected void move(double left, double right) {
      double max=6.2;
      getMotor("left wheel motor").setVelocity(left * max / 100);
      getMotor("right wheel motor").setVelocity(right * max / 100);
    }
    protected double[] readDistanceSensorValues() {
		// read sensors outputs
        double[] psValues = {0, 0, 0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 8 ; i++){
  	psValues[i] = distanceSensor[i].getValue();
        }
        return psValues;
    }
    private boolean handleTargetDetection(List<CameraRecognitionObject> detectedObjects) {
        // Détecter la cible
        CameraRecognitionObject target = targetDetected(detectedObjects);
        if (target != null) {
            double[] targetPosition = target.getPosition();
            double targetX = targetPosition[0];
            double targetY = targetPosition[1];
    
            // Affichage et envoi de message uniquement si la cible est détectée
            System.out.println("Target detected at position: X=" + targetX + ", Y=" + targetY);
            String message = "Target detected at: X=" + targetX + ", Y=" + targetY;
            broadcastMessage(message); // Envoi du message avec les coordonnées de la cible
    
            // Logique de déplacement vers la cible (ajustement de la direction)
            // Si la position du robot est mise à jour
            System.out.println("Robot position: X=" + odometry.getX() + ", Y=" + odometry.getY() + ", Theta=" + odometry.getTheta());
    
            // Transformation de la position de la cible
            double deltaX = targetX - odometry.getX();
            double deltaY = targetY - odometry.getY();
            double angleToTarget = Math.atan2(deltaY, deltaX);
    
            // Log du calcul de l'angle vers la cible
            System.out.println("Angle to target: " + angleToTarget);
            System.out.println("Robot orientation (Theta): " + odometry.getTheta());
    
            // Transformation de la position de la cible du repère robot au repère global
            double targetX_global = odometry.getX() + (deltaX * Math.cos(odometry.getTheta()) - deltaY * Math.sin(odometry.getTheta()));
            double targetY_global = odometry.getY() + (deltaX * Math.sin(odometry.getTheta()) + deltaY * Math.cos(odometry.getTheta()));
    
            // Log de la transformation
            System.out.println("Target in robot's frame: X=" + targetX + ", Y=" + targetY);
            System.out.println("Target in global frame: X=" + targetX_global + ", Y=" + targetY_global);
    
            // Envoi du message avec la position dans le repère global
            String globalMessage = "Target detected at global position: X=" + targetX_global + ", Y=" + targetY_global;
            broadcastMessage(globalMessage);
    
            // Logique de déplacement vers la cible
            if (Math.abs(odometry.getTheta() - angleToTarget) > 0.1) {
                move(-5.0, 25.0); // Ajustement pour tourner vers la cible
                step(timeStep);
            } else {
                move(25.0, 25.0); // Avancer vers la cible
                step(timeStep);
            }
    
            return true; // Indiquer que la cible a été détectée et traitée
        }
    
        return false; // Aucune cible détectée
    }
   
      // Méthode de vérification des messages reçus
      protected String checkMailBox() {
          String message = null;
          
          while (receiver.getQueueLength() > 0) {
              byte[] receivedData = receiver.getData();
              receiver.nextPacket();
              
              if (receivedData != null) {
                  message = new String(receivedData); 
                  System.out.println("Received message: " + message); 
              }
          }
      
          return message; 
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
			(psValues[1] > threshold && psValues[7] > threshold && psValues[6] > threshold)||
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
	
	protected CameraRecognitionObject targetDetected(List<CameraRecognitionObject> detected) {
		for(CameraRecognitionObject ob:detected) {
			if(ob.getModel().compareTo("cible") == 0)
				return ob;
		}
		return null;		
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
