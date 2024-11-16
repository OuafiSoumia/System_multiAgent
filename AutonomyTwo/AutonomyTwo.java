import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.CameraRecognitionObject;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Emitter;
import com.cyberbotics.webots.controller.LED;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Receiver;
import com.cyberbotics.webots.controller.Robot;

public class AutonomyTwo extends Robot {
	private int timeStep;
	private DistanceSensor[] distanceSensor;	
	private Motor leftMotor;
	private Motor rightMotor;
	private Camera camera;
	private Emitter emitter;
	private Receiver receiver;
	private LED[] leds;
	private Random random;
	
	
	public AutonomyTwo() {
            	random = new Random();
		timeStep = 128;  // set the control time step


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

		// Actuators initialization
		// Motors
		leftMotor = this.getMotor("left wheel motor");
		rightMotor = this.getMotor("right wheel motor");
		leftMotor.setPosition(Double.POSITIVE_INFINITY);
		rightMotor.setPosition(Double.POSITIVE_INFINITY);
		leftMotor.setVelocity(0.0);
		rightMotor.setVelocity(0.0);

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

	/**
	 * 
	 * @return a double array with values for each IR sensor 
	 * Each value is between approx. [67 ; 750 (very close - contact)]
	 * (see https://cyberbotics.com/doc/guide/epuck)
	 */
	protected double[] readDistanceSensorValues() {
		// read sensors outputs
		double[] psValues = {0, 0, 0, 0, 0, 0, 0, 0};
		for (int i = 0; i < 8 ; i++){
			psValues[i] = distanceSensor[i].getValue();
		}

		return psValues;
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
	 * Look in a List of camera detected objects if other robots are recognized 
	 * @param detected: a List of camera detected objects
	 * @return a List of CameraRecognitionObject representing the other robots
	 */
	protected List<CameraRecognitionObject> otherRobotsDetected(List<CameraRecognitionObject> detected) {
		ArrayList<CameraRecognitionObject> robots=new ArrayList<>();
		for(CameraRecognitionObject ob:detected) {
			if(ob.getModel().compareTo("e-puck") == 0)
				robots.add(ob);
		}
		return robots;		
	}

	
            /**
             * The main method of the robot behaviour
             */ 
            public void run() {    
                while (step(timeStep) != -1) {    
                    double[] psValues = readDistanceSensorValues();
                    int randomOffset = random.nextInt(-20, 25);
                    double threshold = 100.0;
            
                    List<CameraRecognitionObject> detectedObjects = cameraDetection();
                    CameraRecognitionObject target = targetDetected(detectedObjects);
            
                    if (target != null && handleTargetDetection(target)) {
                        break;
                    }
            
                    if (shouldMoveForward(psValues, threshold)) {
                        move(50 + randomOffset, 50 - randomOffset);
                    } else if (shouldTurnLeft(psValues, threshold)) {
                        move(-50.0, 50.0);
                    } else if (shouldTurnRandom(psValues, threshold)) {
                        moveRandomDirection();
                    } else if (shouldTurnRight(psValues, threshold)) {
                        move(50.0, -50.0);
                    } else {
                        move(-randomOffset, randomOffset);
                    }
                }
            }
            
            /**
             * Handles the logic when a target is detected.
             * @param target The detected target object.
             * @return True if the robot reached the target; false otherwise.
             */
            private boolean handleTargetDetection(CameraRecognitionObject target) {
                double[] targetPosition = target.getPosition();
                double yPosition = targetPosition[1];
                double xPosition = targetPosition[0];
            
                if (yPosition < 0.1 && yPosition > 0) {
                    System.out.println("x est -------------");
                    move(-5.0, 25.0); // Turn left
                } else if (yPosition > -0.1 && yPosition <= 0) {
                    System.out.println("x est +++++++++");
                    move(25.0, -5.0); // Turn right
                } else if (xPosition < 0.1) {
                    System.out.println("else **************************");
                    move(0, 0);
                    return true; // Target reached
                }
            
                return false;
            }
            
            /**
             * Determines if the robot should move forward.
             */
            private boolean shouldMoveForward(double[] psValues, double threshold) {
                return psValues[0] < threshold && psValues[7] < threshold;
            }
            
            /**
             * Determines if the robot should turn left.
             */
            private boolean shouldTurnLeft(double[] psValues, double threshold) {
                return psValues[2] > threshold ||
                       (psValues[0] > threshold && psValues[1] > threshold && psValues[2] > threshold) ||
                       (psValues[0] > threshold && psValues[1] > threshold && psValues[7] > threshold) ||
                       (psValues[0] > threshold && psValues[1] > threshold) ||
                       (psValues[1] > threshold && psValues[2] > threshold);
            }
            
            /**
             * Determines if the robot should turn in a random direction.
             */
            private boolean shouldTurnRandom(double[] psValues, double threshold) {
                return (psValues[0] > threshold && psValues[7] > threshold) || 
                       (psValues[0] > threshold && psValues[7] > threshold && psValues[1] > threshold && psValues[6] > threshold) ||
                       (psValues[1] > threshold && psValues[7] > threshold && psValues[6] > threshold);
            }
            
            /**
             * Moves the robot in a random direction.
             */
            private void moveRandomDirection() {
                int direction = random.nextInt(2);
                if (direction == 1) {
                    move(-50.0, 50.0); // Turn left
                } else {
                    move(50.0, -50.0); // Turn right
                }
            }
            
            /**
             * Determines if the robot should turn right.
             */
            private boolean shouldTurnRight(double[] psValues, double threshold) {
                return psValues[5] > threshold ||
                       (psValues[5] > threshold && psValues[6] > threshold) ||
                       (psValues[6] > threshold && psValues[7] > threshold) ||
                       (psValues[5] > threshold && psValues[6] > threshold && psValues[7] > threshold) ||
                       (psValues[0] > threshold && psValues[6] > threshold && psValues[7] > threshold);
            }


	public static void main(String[] args) {
		AutonomyTwo controller = new AutonomyTwo();
		controller.run();
		  System.out.println("============ done");
		  
	}
}
