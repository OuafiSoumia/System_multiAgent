import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.CameraRecognitionObject;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.LED;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;

public class AutonomyTwo extends Robot {
    private int timeStep;
    private DistanceSensor[] distanceSensor;        
    private Motor leftMotor;
    private Motor rightMotor;
    private Camera camera;
    private LED[] leds;
    private Random random;
    private List<String> visitedPositions; // Liste pour mémoriser les positions visitées
    
    public AutonomyTwo() {
        random = new Random();
        timeStep = 128;  // set the control time step
        
        // Initialisation des capteurs IR
        distanceSensor = new DistanceSensor[8];
        String[] sensorNames = {
            "ps0", "ps1", "ps2", "ps3",
            "ps4", "ps5", "ps6", "ps7"
        };
        for (int i = 0; i < 8; i++) {
            distanceSensor[i] = this.getDistanceSensor(sensorNames[i]);
            distanceSensor[i].enable(timeStep);
        }

        // Initialisation de la caméra
        camera = this.getCamera("camera");
        camera.enable(timeStep);
        camera.recognitionEnable(timeStep);

        // Initialisation des moteurs
        leftMotor = this.getMotor("left wheel motor");
        rightMotor = this.getMotor("right wheel motor");
        leftMotor.setPosition(Double.POSITIVE_INFINITY);
        rightMotor.setPosition(Double.POSITIVE_INFINITY);
        leftMotor.setVelocity(0.0);
        rightMotor.setVelocity(0.0);

        // Initialisation des LED
        leds = new LED[10];
        String[] ledsNames = {
            "led0", "led1", "led2", "led3",
            "led4", "led5", "led6", "led7",
            "led8", "led9"
        };
        for (int i = 0; i < 10; i++) {
            leds[i] = this.getLED(ledsNames[i]);
        }

        // Initialisation de la liste des positions visitées
        visitedPositions = new ArrayList<>();
    }

    protected double[] readDistanceSensorValues() {
        double[] psValues = {0, 0, 0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 8 ; i++) {
            psValues[i] = distanceSensor[i].getValue();
        }
        return psValues;
    }

    protected void move(double left, double right) {
        double max = 6.2;
        getMotor("left wheel motor").setVelocity(left * max / 100);
        getMotor("right wheel motor").setVelocity(right * max / 100);
    }

    protected void setLED(int num, boolean on) {
        if(num < 10) {
            leds[num].set(on ? 1 : 0);
        }
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

    protected List<CameraRecognitionObject> otherRobotsDetected(List<CameraRecognitionObject> detected) {
        ArrayList<CameraRecognitionObject> robots = new ArrayList<>();
        for (CameraRecognitionObject ob : detected) {
            if (ob.getModel().compareTo("e-puck") == 0)
                robots.add(ob);
        }
        return robots;                
    }

    // Fonction pour vérifier si une position a déjà été visitée
    protected boolean hasVisitedPosition(double x, double y) {
        String positionKey = x + "," + y;
        return visitedPositions.contains(positionKey);
    }

    // Fonction pour ajouter une position à la liste des positions visitées
    protected void addVisitedPosition(double x, double y) {
        String positionKey = x + "," + y;
        if (!visitedPositions.contains(positionKey)) {
            visitedPositions.add(positionKey);
        }
    }

    public void run() {        
        int aleatoire = random.nextInt(25, 50); // Valeur aléatoire entre 25 et 50

        // Boucle principale de contrôle
        while (step(timeStep) != -1) {        
            double[] psValues = readDistanceSensorValues();
            List<CameraRecognitionObject> detectedObjects = cameraDetection();
            CameraRecognitionObject target = targetDetected(detectedObjects);
            
            // Vérification de la présence de la cible
            if (target != null) {
                double[] targetPosition = target.getPosition();
                double xPosition = targetPosition[0];
                double yPosition = targetPosition[1];

                // Déplacer le robot en fonction de la position de la cible
                if (yPosition < 0.1 && yPosition > 0) {
                    move(-5.0, 25.0); // tourner à gauche
                } else if (yPosition > -0.1 && yPosition <= 0) {
                    move(25.0, -5.0); // tourner à droite
                }
                if (xPosition < 0.1) {
                    move(0, 0); // Arrêter le robot si la cible est atteinte
                    break;
                }
                this.step(timeStep); // Actualisation de l'état du robot
            }
            
            // Gestion des obstacles
            double threshold = 100.0;
            if (psValues[0] < threshold && psValues[7] < threshold) {
                move(50 + aleatoire, 50 - aleatoire); // Avancer en ligne droite
            } else if ((psValues[2] > threshold) || (psValues[0] > threshold && psValues[1] > threshold && psValues[2] > threshold) ||
                       (psValues[0] > threshold && psValues[1] > threshold && psValues[7] > threshold)) {
                move(-50.0, 50.0);  // tourner à droite pour éviter l'obstacle
            } else if ((psValues[0] > threshold && psValues[7] > threshold) ||
                       (psValues[0] > threshold && psValues[1] > threshold && psValues[7] > threshold)) {
                int direction = random.nextInt(2);
                if (direction == 1) {
                    move(-50.0, 50.0);
                } else {
                    move(50.0, -50.0);
                }
            } else if ((psValues[5] > threshold) || (psValues[5] > threshold && psValues[6] > threshold)) {
                move(50.0, -50.0); // tourner à gauche pour éviter l'obstacle
            } else {
                move(-aleatoire, aleatoire);  // Déplacement aléatoire
            }
            
            // Enregistrer la position du robot si elle n'a pas été visitée
            double x = getPosition()[0];  // Récupérer la position x du robot
            double y = getPosition()[1];  // Récupérer la position y du robot
            
            if (!hasVisitedPosition(x, y)) {
                addVisitedPosition(x, y);
            }
        }
    }

    public static void main(String[] args) {
        AutonomyTwo controller = new AutonomyTwo();
        controller.run();
        System.out.println("============ done");
    }
}