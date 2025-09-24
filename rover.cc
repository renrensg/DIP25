#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <math.h>

//////////////////// Pins (adjust when assembled)
const int PUMP_FRONT_PIN = 6;     // Relay for front pump
const int PUMP_BACK_PIN  = 7;     // Relay for back pump
const int COLLECT_DONE_PIN = 5;   // Input from Pi (HIGH = done)
const int MOTOR_L_FWD = 9;        // Motor driver pins
const int MOTOR_L_REV = 10;
const int MOTOR_R_FWD = 11;
const int MOTOR_R_REV = 12;

//////////////////// Serial ports
// GPS on pins 2=RX, 3=TX
SoftwareSerial gpsSS(2, 3);
TinyGPSPlus gps;

// HC-12 radio on pins 4=RX, 8=TX
SoftwareSerial radio(4, 8);

//////////////////// Config
const double ARRIVE_RADIUS_M = 2.0;         // distance tolerance
const unsigned long FRONT_PUMP_TIMEOUT_MS = 20000;
const unsigned long BACK_PUMP_DRIVE_MS   = 8000;
const int DRIVE_PWM = 150;                  // motor speed
const int TURN_PWM  = 140;

//////////////////// State machine
enum State { IDLE, GOTO, ARRIVED, COLLECT_FRONT, FORWARD_WITH_BACK_PUMP, DONE };
State state = IDLE;

double targetLat = 0.0, targetLon = 0.0;
bool hasTarget = false;
unsigned long stateStart = 0;

//////////////////// Utils
static double toRad(double d){ return d * M_PI / 180.0; }

double haversine_m(double lat1, double lon1, double lat2, double lon2){
  double dLat = toRad(lat2 - lat1);
  double dLon = toRad(lon2 - lon1);
  double a = sin(dLat/2)*sin(dLat/2) +
             cos(toRad(lat1))*cos(toRad(lat2))*sin(dLon/2)*sin(dLon/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  return 6371000.0 * c;
}

//////////////////// Motor control (simple differential drive)
void drive_stop(){
  analogWrite(MOTOR_L_FWD, 0); analogWrite(MOTOR_L_REV, 0);
  analogWrite(MOTOR_R_FWD, 0); analogWrite(MOTOR_R_REV, 0);
}
void drive_forward(){
  analogWrite(MOTOR_L_FWD, DRIVE_PWM); analogWrite(MOTOR_L_REV, 0);
  analogWrite(MOTOR_R_FWD, DRIVE_PWM); analogWrite(MOTOR_R_REV, 0);
}

//////////////////// Pumps
void pump_front(bool on){ digitalWrite(PUMP_FRONT_PIN, on ? HIGH : LOW); }
void pump_back (bool on){ digitalWrite(PUMP_BACK_PIN,  on ? HIGH : LOW);  }

//////////////////// Setup
void setup(){
  pinMode(PUMP_FRONT_PIN, OUTPUT);
  pinMode(PUMP_BACK_PIN, OUTPUT);
  pinMode(COLLECT_DONE_PIN, INPUT_PULLDOWN); // HIGH when Pi says done

  pinMode(MOTOR_L_FWD, OUTPUT); pinMode(MOTOR_L_REV, OUTPUT);
  pinMode(MOTOR_R_FWD, OUTPUT); pinMode(MOTOR_R_REV, OUTPUT);
  drive_stop(); pump_front(false); pump_back(false);

  Serial.begin(115200);
  gpsSS.begin(9600);
  radio.begin(9600);

  Serial.println(F("[ROVER] Ready"));
}

//////////////////// Radio packet parsing
bool parseGeoPacket(String line, double &lat, double &lon){
  if (!line.startsWith("GEO,")) return false;
  int p1 = line.indexOf(',', 4);
  int p2 = line.indexOf(',', p1+1);
  if (p1 < 0 || p2 < 0) return false;
  lat = line.substring(4, p1).toFloat();
  lon = line.substring(p1+1, p2).toFloat();
  return true;
}

void readRadio(){
  static String buf;
  while (radio.available()){
    char c = radio.read();
    if (c == '\n'){
      buf.trim();
      double la, lo;
      if (parseGeoPacket(buf, la, lo)){
        targetLat = la; targetLon = lo; hasTarget = true;
        state = GOTO; stateStart = millis();
        Serial.print(F("[ROVER] Target set: ")); Serial.print(la, 6); Serial.print(", "); Serial.println(lo, 6);
      }
      buf = "";
    } else {
      buf += c;
    }
  }
}

//////////////////// GPS
void readGPS(){
  while (gpsSS.available()){
    gps.encode(gpsSS.read());
  }
}

double currentLat(){ return gps.location.isValid() ? gps.location.lat() : NAN; }
double currentLon(){ return gps.location.isValid() ? gps.location.lng() : NAN; }

//////////////////// Main loop
void loop(){
  readRadio();
  readGPS();

  double lat = currentLat();
  double lon = currentLon();

  switch (state){
    case IDLE:
      drive_stop(); pump_front(false); pump_back(false);
      break;

    case GOTO:
      if (!hasTarget || isnan(lat) || isnan(lon)) { drive_stop(); break; }
      {
        double dist = haversine_m(lat, lon, targetLat, targetLon);
        if (dist <= ARRIVE_RADIUS_M){
          drive_stop();
          state = ARRIVED; stateStart = millis();
          Serial.println(F("[ROVER] Arrived at target"));
        } else {
          drive_forward();
        }
      }
      break;

    case ARRIVED:
      if (millis() - stateStart > 1000){
        state = COLLECT_FRONT; stateStart = millis();
        pump_front(true);
        Serial.println(F("[ROVER] Front pump ON"));
      }
      break;

    case COLLECT_FRONT:
      if (digitalRead(COLLECT_DONE_PIN) || (millis() - stateStart > FRONT_PUMP_TIMEOUT_MS)){
        pump_front(false);
        Serial.println(F("[ROVER] Front pump OFF (collection done)"));
        pump_back(true);
        drive_forward();
        state = FORWARD_WITH_BACK_PUMP; stateStart = millis();
      }
      break;

    case FORWARD_WITH_BACK_PUMP:
      if (millis() - stateStart > BACK_PUMP_DRIVE_MS){
        drive_stop(); pump_back(false);
        state = DONE;
        Serial.println(F("[ROVER] Back pump OFF, mission DONE"));
      }
      break;

    case DONE:
      // stay here or reset to IDLE
      break;
  }
}
