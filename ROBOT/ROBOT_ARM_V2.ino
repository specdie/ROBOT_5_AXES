#include <Servo.h>
#include <math.h>

// Définition des pins des servomoteurs
#define BASE_SERVO_PIN 8
#define SHOULDER_SERVO_PIN 9
#define ELBOW_SERVO_PIN 10
#define WRIST_SERVO_PIN 11
#define GRIPPER_SERVO_PIN 12

// Longueurs des segments du bras robot
const float L1 = 10.5; // Longueur du segment de Axe1
const float L2 = 11.3; // Longueur du segment de Axe2
const float L3 = 11.0; // Longueur du segment de Axe3
const float L4 = 5.0;  // Longueur du segment de Axe4
const float L5 = 5.0;  // Longueur du segment de Axe5

// Création des objets Servo pour chaque servo moteur
Servo baseServo;
Servo shoulderServo;
Servo elbowServo;
Servo wristServo;
Servo gripperServo;

// Fonction pour déplacer le bras robot en fonction des coordonnées XYZ
void moveArm(float x, float y, float z) {
  // Calcul des angles nécessaires pour chaque servo moteur
  float baseAngle, shoulderAngle, elbowAngle, wristAngle, gripperAngle;
  inverseKinematics(x, y, z, baseAngle, shoulderAngle, elbowAngle, wristAngle, gripperAngle);

  // Déplacement des servomoteurs aux angles calculés
  baseServo.write(baseAngle);
  shoulderServo.write(shoulderAngle);
  elbowServo.write(elbowAngle);
  wristServo.write(wristAngle);
  gripperServo.write(gripperAngle);

  // Attente pour que le bras robot atteigne sa position
  delay(2000);
}

// Fonction de calcul de la cinématique inverse
void inverseKinematics(float x, float y, float z, float &baseAngle, float &shoulderAngle, float &elbowAngle, float &wristAngle, float &gripperAngle) {
  // Calcul de la position du poignet (x, y, z) par rapport à l'origine de la base
  float x4 = x - L5; // Décalage pour tenir compte de la longueur du segment de la pince
  float y4 = y;
  float z4 = z;

  // Calcul de l'angle du servo de base (rotation autour de l'axe Z)
  baseAngle = atan2(y4, x4) * 180 / M_PI;

  // Calcul de la distance projetée (distance horizontale) entre l'axe de la base et le poignet
  float r = sqrt(x4 * x4 + y4 * y4);

  // Calcul de l'angle entre le segment d'épaule et le plan horizontal
  float phi2 = atan2(z4 - L1, r);

  // Calcul de la distance projetée entre l'axe de la base et le point de rotation du coude
  float r2 = sqrt((z4 - L1) * (z4 - L1) + r * r);

  // Calcul de l'angle du servo d'épaule (rotation autour de l'axe Y)
  shoulderAngle = atan2(z4 - L1, r) * 180 / M_PI + acos((L2 * L2 + r2 * r2 - L3 * L3) / (2 * L2 * r2)) * 180 / M_PI;

  // Calcul de l'angle du servo de coude (rotation autour de l'axe Y)
  elbowAngle = acos((L2 * L2 + L3 * L3 - r2 * r2) / (2 * L2 * L3)) * 180 / M_PI;

  // Calcul de l'angle du servo de poignet (rotation autour de l'axe Y)
  wristAngle = -(shoulderAngle + elbowAngle - 180);

  // Angle de la pince
  gripperAngle = 90; 
}

void setup() {
  // Initialisation des servomoteurs
  baseServo.attach(BASE_SERVO_PIN);
  shoulderServo.attach(SHOULDER_SERVO_PIN);
  elbowServo.attach(ELBOW_SERVO_PIN);
  wristServo.attach(WRIST_SERVO_PIN);
  gripperServo.attach(GRIPPER_SERVO_PIN);

  // Initialisation de la communication série // Raspberry
  Serial.begin(9600);
  Serial.println("Bras robot prêt !");
}

void loop() {
  // Mouvement du bras robot vers une position spécifique

  // Attendre les coordonnées XYZ depuis le Raspeberry
  if (Serial.available() >= 12) { // Attendre au moins 12 caractères (3 valeurs flottantes séparées par des virgules)
    String input = Serial.readStringUntil('\n');
    input.trim();
    int commaIndex1 = input.indexOf(',');
    int commaIndex2 = input.indexOf(',', commaIndex1 + 1);
    
    if (commaIndex1 != -1 && commaIndex2 != -1) {
      String xStr = input.substring(0, commaIndex1);
      String yStr = input.substring(commaIndex1 + 1, commaIndex2);
      String zStr = input.substring(commaIndex2 + 1);
      
      float x = xStr.toFloat();
      float y = yStr.toFloat();
      float z = zStr.toFloat();

      moveArm(x, y, z); // Déplacer le bras robot aux coordonnées 
      delay(700); // Attente de 5 secondes avant de passer à la prochaine commande
    }
  }

  //Etablissement du Pick and place sans la caméra
  int TabPosition[4][3] = {
    {10,10,10},
    {15,15,15},
    {10,10,10},
    {10,0,0},
    };

  // tableau qui crée la trajectoire
  for (int t=0; t<4; t++)
  {
    //Serial.println(TabPosition[t]);
    moveArm(TabPosition[t][0], TabPosition[t][1], TabPosition[t][2]); // Déplacer le bras robot aux coordonnées (10, 20, 15)

    if (t == 1)
    {
      gripperServo.write(180);
      delay(3000);
    }
    else if(t == 3)
    {
      gripperServo.write(0);
      delay(3000);
    }
    else if (t==0)
    {
      gripperServo.write(0);
      delay(3000);
    }
    delay(700); // Attente de 5 secondes avant de passer à la prochaine commande
  }
}
