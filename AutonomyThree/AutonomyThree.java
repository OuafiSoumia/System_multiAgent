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
import java.util.Locale;  // Ajouter cet import en haut du fichier

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
    
    // Nouveaux attributs pour la coordination
    private boolean targetReached = false;
    private double targetGlobalX = 0;
    private double targetGlobalY = 0;
    private boolean movingToSharedTarget = false;
        // Ajuster le seuil de la cible atteinte
    private static final double TARGET_REACHED_THRESHOLD = 0.15; // Augmenté pour plus de tolérance
    private static final double MOVEMENT_SPEED_FACTOR = 1.5;    // Facteur pour ajuster la vitesse générale

    public AutonomyThree() {
        random = new Random();
        timeStep = 128;
        odometry = new Odometry();

        // Sensors initialization
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

        // Motors
        leftMotor = this.getMotor("left wheel motor");
        rightMotor = this.getMotor("right wheel motor");
        leftMotor.setPosition(Double.POSITIVE_INFINITY);
        rightMotor.setPosition(Double.POSITIVE_INFINITY);
        leftMotor.setVelocity(0.0);
        rightMotor.setVelocity(0.0);

        // Motor sensors
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


    private void moveRandomly() {
        double leftSpeed = random.nextDouble() * 100 - 50;
        double rightSpeed = random.nextDouble() * 100 - 50;
        move(leftSpeed, rightSpeed);
    }
    public void run() {
        // Initialiser l'odométrie et vérifier l'initialisation
        double leftInitial = leftMotorSensor.getValue();
        double rightInitial = rightMotorSensor.getValue();
        
        //System.out.println("Initial sensor values - Left: " + leftInitial + ", Right: " + rightInitial);
        odometry.track_start_pos(leftInitial, rightInitial);
       // System.out.println("Odometry initialized");

        while (step(timeStep) != -1) {
            // Mettre à jour et vérifier l'odométrie
            double leftCurrent = leftMotorSensor.getValue();
            double rightCurrent = rightMotorSensor.getValue();
            
            // Debug odométrie
            if (leftCurrent != 0 || rightCurrent != 0) {
              //  System.out.println("Current sensor values - Left: " + leftCurrent + ", Right: " + rightCurrent);
            }
            
            odometry.track_step_pos(leftCurrent, rightCurrent);
            
            // Vérifier l'état de l'odométrie
            double robotX = odometry.getX();
            double robotY = odometry.getY();
            double robotTheta = odometry.getTheta();
            
            if (!Double.isNaN(robotX) && !Double.isNaN(robotY) && !Double.isNaN(robotTheta)) {
                //System.out.println("Robot position - X: " + robotX + ", Y: " + robotY + ", Theta: " + robotTheta);
            }
            
            double[] psValues = readDistanceSensorValues();
            List<CameraRecognitionObject> detectedObjects = cameraDetection();

            String receivedMessage = checkMailBox();
            if (receivedMessage != null) {
                System.out.println("Received message: " + receivedMessage);
            }

            if (handleSimpleTargetDetection(detectedObjects)) continue;

            if (handleObstacleAvoidance(psValues)) continue;

            if (!movingToSharedTarget) {
                moveRandomly();
            }
        }
    }private double[] calculateGlobalCoordinates(double localX, double localY) {
        double robotX = odometry.getX();
        double robotY = odometry.getY();
        double robotTheta = Math.toRadians(odometry.getTheta());

        // Transformation de coordonnées avec matrice de rotation
        double globalX = robotX + (localX * Math.cos(robotTheta) - localY * Math.sin(robotTheta));
        double globalY = robotY + (localX * Math.sin(robotTheta) + localY * Math.cos(robotTheta));

        return new double[] {globalX, globalY};
    }
private boolean handleTargetDetection(List<CameraRecognitionObject> detectedObjects) {
        if (movingToSharedTarget) {
            return moveToSharedTarget();
        }

        CameraRecognitionObject target = targetDetected(detectedObjects);
        if (target != null) {
            // Positions locales par rapport au robot
            double[] localPosition = target.getPosition();
            double localX = localPosition[0]; // avant/arrière
            double localY = localPosition[1]; // gauche/droite

            if (Double.isNaN(localX) || Double.isNaN(localY)) {
                System.out.println("Invalid target coordinates detected");
                return false;
            }

            // Position et orientation actuelles du robot
            double robotX = odometry.getX();
            double robotY = odometry.getY();
            double robotTheta = Math.toRadians(odometry.getTheta());

            // Conversion en coordonnées globales
            double targetX_global = robotX + (localX * Math.cos(robotTheta) - localY * Math.sin(robotTheta));
            double targetY_global = robotY + (localX * Math.sin(robotTheta) + localY * Math.cos(robotTheta));

            // Log détaillé
            System.out.println("Robot at: X=" + robotX + ", Y=" + robotY + ", Theta=" + Math.toDegrees(robotTheta));
            System.out.println("Target local: X=" + localX + ", Y=" + localY);
            System.out.println("Target global: X=" + targetX_global + ", Y=" + targetY_global);

            // Distance locale pour la détection
            double distanceToTarget = Math.sqrt(localX * localX + localY * localY);
            
            if (distanceToTarget < TARGET_REACHED_THRESHOLD && !targetReached) {
                targetReached = true;
                
                // Envoyer la position globale calculée
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

    private boolean moveToSharedTarget() {
        // Position actuelle du robot
        double robotX = odometry.getX();
        double robotY = odometry.getY();
        double robotTheta = Math.toRadians(odometry.getTheta());

        // Vecteur vers la cible en coordonnées globales
        double deltaX = targetGlobalX - robotX;
        double deltaY = targetGlobalY - robotY;

        // Conversion en coordonnées locales pour la navigation
        double localX = deltaX * Math.cos(-robotTheta) - deltaY * Math.sin(-robotTheta);
        double localY = deltaX * Math.sin(-robotTheta) + deltaY * Math.cos(-robotTheta);

        // Distance à la cible
        double distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        
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
        if (Math.abs(angleToTarget) > Math.PI/6) { // Plus de 30 degrés
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
    }private boolean handleSimpleTargetDetection(List<CameraRecognitionObject> detectedObjects) {
        if (movingToSharedTarget) {
            return moveToSharedTarget();
        }

        CameraRecognitionObject target = targetDetected(detectedObjects);
        if (target != null) {
            // Positions locales par rapport au robot
            double[] localPosition = target.getPosition();
            double localX = localPosition[0]; // avant/arrière
            double localY = localPosition[1]; // gauche/droite

            if (Double.isNaN(localX) || Double.isNaN(localY)) {
                System.out.println("Invalid target coordinates detected");
                return false;
            }

            // Position et orientation actuelles du robot
            double robotX = odometry.getX();
            double robotY = odometry.getY();
            double robotTheta = Math.toRadians(odometry.getTheta());

            // Conversion en coordonnées globales
            double targetX_global = robotX + (localX * Math.cos(robotTheta) - localY * Math.sin(robotTheta));
            double targetY_global = robotY + (localX * Math.sin(robotTheta) + localY * Math.cos(robotTheta));

            // Log détaillé
            System.out.println("Robot at: X=" + robotX + ", Y=" + robotY + ", Theta=" + Math.toDegrees(robotTheta));
            System.out.println("Target local: X=" + localX + ", Y=" + localY);
            System.out.println("Target global: X=" + targetX_global + ", Y=" + targetY_global);

            // Distance locale pour la détection
            double distanceToTarget = Math.sqrt(localX * localX + localY * localY);
            
            if (distanceToTarget < TARGET_REACHED_THRESHOLD && !targetReached) {
                targetReached = true;
                
                // §§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§
                String targetReachedMessage = String.format(Locale.US, "TARGET_REACHED:%.6f:%.6f", 
                    robotX, robotX);
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


 
   // Modifier la méthode move pour avoir un contrôle plus précis
    protected void move(double left, double right) {
        double max = 6.2;
        
        // Ajouter une accélération progressive
        double currentLeft = leftMotor.getVelocity();
        double currentRight = rightMotor.getVelocity();
        
        double targetLeft = left * max / 100;
        double targetRight = right * max / 100;
        
        // Limiter le changement de vitesse
        double maxChange = max * 0.1; // 10% de la vitesse max par pas
        
        double newLeft = currentLeft + Math.max(-maxChange, Math.min(maxChange, targetLeft - currentLeft));
        double newRight = currentRight + Math.max(-maxChange, Math.min(maxChange, targetRight - currentRight));
        
        leftMotor.setVelocity(newLeft);
        rightMotor.setVelocity(newRight);
    }private boolean targetBroadcasted = false; // Nouvel attribut

    

    protected String checkMailBox() {
        String message = null;
        
        while (receiver.getQueueLength() > 0) {
            byte[] receivedData = receiver.getData();
            receiver.nextPacket();
            
            if (receivedData != null) {
                message = new String(receivedData);
                
                if (message.startsWith("TARGET_REACHED:") && !targetReached) {
                    String[] parts = message.split(":");
                    if (parts.length == 3) {
                        try {
                            String xStr = parts[1].replace(',', '.');
                            String yStr = parts[2].replace(',', '.');
                            
                            double newTargetX = Double.parseDouble(xStr);
                            double newTargetY = Double.parseDouble(yStr);
                            
                            if (!Double.isNaN(newTargetX) && !Double.isNaN(newTargetY)) {
                                targetGlobalX = newTargetX;
                                targetGlobalY = newTargetY;
                                if (!movingToSharedTarget) {
                                    System.out.println("Starting movement to shared target at: X=" + targetGlobalX + ", Y=" + targetGlobalY);
                                    movingToSharedTarget = true;
                                }
                            }
                        } catch (NumberFormatException e) {
                            System.out.println("Error parsing coordinates - Raw values: X=" + parts[1] + ", Y=" + parts[2]);
                        }
                    }
                }
            }
        }
        return message;
    }

    protected void broadcastMessage(String message) {
        System.out.println("Sending message: " + message);
        emitter.send(message.getBytes());
    }



    protected double[] readDistanceSensorValues() {
        double[] psValues = new double[8];
        for (int i = 0; i < 8; i++) {
            psValues[i] = distanceSensor[i].getValue();
        }
        return psValues;
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

    protected List<CameraRecognitionObject> cameraDetection() {
        ArrayList<CameraRecognitionObject> detected = new ArrayList<>();
        int nb = camera.getRecognitionNumberOfObjects();
        if (nb > 0) {
            CameraRecognitionObject[] objects = camera.getRecognitionObjects();
            for (int i = 0; i < objects.length; i++) {
                detected.add(objects[i]);
            }
        }
        return detected;
    }

    protected CameraRecognitionObject targetDetected(List<CameraRecognitionObject> detected) {
        for (CameraRecognitionObject ob : detected) {
            if (ob.getModel().compareTo("cible") == 0)
                return ob;
        }
        return null;
    }

    public static void main(String[] args) {
        AutonomyThree controller = new AutonomyThree();
        controller.run();
    }

    private class Odometry {
        private double wheel_distance;
        private double wheel_conversion_left;
        private double wheel_conversion_right;
        private double pos_left_prev;
        private double pos_right_prev;
        private double x;
        private double y;
        private double theta;

        private double increments_per_tour = 1000.0;
        private double axis_wheel_ratio = 1.4134;
        private double wheel_diameter_left = 0.0416;
        private double wheel_diameter_right = 0.0416;
        private double scaling_factor = 0.976;

        public Odometry() {}

        public int track_start_pos(double pos_left, double pos_right) {
            x = 0;
            y = 0;
            theta = 0;

            pos_left_prev = pos_left;
            pos_right_prev = pos_right;

            wheel_distance = axis_wheel_ratio * scaling_factor * (wheel_diameter_left + wheel_diameter_right) / 2;
            wheel_conversion_left = wheel_diameter_left * scaling_factor * Math.PI / increments_per_tour;
            wheel_conversion_right = wheel_diameter_right * scaling_factor * Math.PI / increments_per_tour;

            return 1;
        }

      

        public double getX() { return x; }
        public void setX(double x) { this.x = x; }
        public double getY() { return y; }
       public void setY(double y) { 
            this.y = y; 
        }

        public double getTheta() { 
            return theta; 
        }

        public void setTheta(double theta) { 
            this.theta = theta; 
        }
              public void track_step_pos(double pos_left, double pos_right) {
            // Vérification des entrées
            if (Double.isNaN(pos_left) || Double.isNaN(pos_right)) {
                System.out.println("Warning: Invalid sensor values detected");
                return;
            }

            double delta_pos_left = pos_left - pos_left_prev;
            double delta_pos_right = pos_right - pos_right_prev;
            
            // Vérification des deltas
            if (Math.abs(delta_pos_left) > 1000 || Math.abs(delta_pos_right) > 1000) {
                System.out.println("Warning: Large position change detected");
                return;
            }

            double delta_left = delta_pos_left * wheel_conversion_left;
            double delta_right = delta_pos_right * wheel_conversion_right;
            double delta_theta = (delta_right - delta_left) / wheel_distance;
            double theta2 = theta + delta_theta * 0.5;
            
            // Calcul des déplacements
            double delta_x = (delta_left + delta_right) * 0.5 * Math.cos(theta2);
            double delta_y = (delta_left + delta_right) * 0.5 * Math.sin(theta2);

            // Mise à jour des positions avec vérification
            if (!Double.isNaN(delta_x) && !Double.isNaN(delta_y) && !Double.isNaN(delta_theta)) {
                x += delta_x;
                y += delta_y;
                theta += delta_theta;

                // Normalisation de theta
                while (theta < 0) theta += 2 * Math.PI;
                while (theta >= 2 * Math.PI) theta -= 2 * Math.PI;
            }

            pos_left_prev = pos_left;
            pos_right_prev = pos_right;
        }
    }
}