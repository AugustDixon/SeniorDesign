#include <ArduinoSTL.h>
#include <Servo.h>

using namespace std;


//Global Constants
#define SignalPin 6             //Input pin that receives signal from Start and Stop buttons
#define OutputHigh 7
#define FinishingLightPin 4      //Finishing Light Pin
#define TowerRaiseServoPin 3     //Tower Raise/Lower Servo Pin
#define TowerRotateServoPin 2    //Tower Rotation Servo Pin
#define TowerRaisedDegree 90     //Degree at which Camera Arm is raised
#define TowerLoweredDegree 180   //Degree at which Camera Arm is lowered
#define RotationSpeed 95         //Degree speed at which Tower Rotation Servo rotates
#define ServoMid 1634
//+ = ccw, - = cw for positional servo
// = ccw,  = cw for continuous servo


//Contains Drone data
class Drone {
private:
  int number;             //Drone ID number
  char numChar;           //Drone ID number as char
  char machineID;         //Drone Machine ID number
public:
  Drone(){
    this->number = 0;
    this->numChar = '0';
    this->machineID = 0;
  }
  Drone(int n, char id){
    this->number = n;
    this->numChar = '0' + n;
    this->machineID = id;
  }
  ~Drone(){  }

  int getNum() {  return this->number; }
  char getNumChar() { return this->numChar;  }
  char getID()  { return this->machineID;  }

  void setNum(int i){
    this->number = i;
    this->numChar = '0' + i;
  }
};


//Class for wrapping serial communication messages
class Message {
  public:
    char source;
    char dest;
    String message;
    
    Message(char s, char d, String m){
      this->source = s;
      this->dest = d;
      this->message = m;
    }
    Message(char d, String m){
      Message('a', d, m);
    }
    Message(){
      this->source = ' ';
      this->dest = ' ';
      this->message = "";
    }
    ~Message(){  }

    //Writes String object to buffer
    //  Arguments:
    //    String data - Data to send
    void serialString(String data){
      for(int i = 0; i < data.length(); ++i)
        Serial.write(data[i]);
    }
    
    //Receives a message regardless of destination while blocking execution
    //  Returns - Message - message received
    Message receiveAnyMessageBlocking(){
      while(true){
        while(Serial.available() > 0)
          if(Serial.read() == 's'){
            String input = Serial.readStringUntil('x');
            Message output = Message(input[0], input[1], input.substring(2, input.length() - 3));
            return output;
          }
        delay(20);
      }
    }

    //Receives a message while blocking execution
    //  Returns - Message - message received
    Message receiveMessageBlocking(){
      while(true){
        while(Serial.available() > 0)
          if(Serial.read() == 's'){
            String input = Serial.readStringUntil('x');
            if(input[1] == 'a'){
              Message output = Message(input[0], input[1], input.substring(2, input.length() - 3));
              return output;
            }
          }
        delay(20);
      }
    }

    //Receives message without blocking execution
    //  Return - Message or NULL
    Message receiveMessage(){
      if(Serial.available() > 0) {
        while(Serial.read() != 's')
          if(Serial.available() == 0)
            return Message();
        String input = Serial.readStringUntil('x');
        if(input[1] == 'a' || input[1] == 'A'){
          Message output = Message(input[0], input[1], input.substring(2, input.length() - 3));
          return output;
        }
      }
    }

    //Returns boolean of message equality
    bool equal(Message b){
      if(this->source == b.source && this->dest == b.dest && this->message == b.message)
        return true;
      else
        return false;
    }

    //Sends a message through Serial
    //  Arguments:
    //    char d - Destination character
    //    String m - Message
    bool sendMessage(char d, String m){
      this->serialString(String("sa" + d + m + "x"));
      Serial.flush();
    }

    //Returns boolean comparison to NULL
    bool isNull(){
      return (this == NULL) || (this->source == ' ' && this->dest == ' ' && this->message == "");
    }
};

Message messHandl;



//Handles serial communications functions
class SerialCom {
public:
  //Constructor begins and waits for serial communications
  SerialCom(){  }
  ~SerialCom(){  }


  //Function to transmit Instruction to selected drone
  //  Arguments:
  //    char dest - Destination character
  void transmitInstruction(char dest){
    messHandl.sendMessage(dest, "I");
    byte temp[8];
    while(true){
      Serial.readBytes(temp, 8);
      Serial.write(temp, 8);
      if((char) temp[7] == 'E')
        break;
    }
  }

  //Function to transmit a binary image file in chunks of 32 bytes
  //  Arguments:
  //    int size - Size of file in bytes
  void transmitBinaryImage(int size){
    messHandl.sendMessage('p', "L");
    messHandl.sendMessage('p', String(size));
    int remainder = size % 32;
    byte temp[32];
    for(int i = 0; i < size; i += 32){
      Serial.readBytes(temp, 32);
      Serial.write(temp, 32);
    }
    if(remainder > 0){
      Serial.readBytes(temp, remainder);
      Serial.write(temp, remainder);
    }
  }

  //Function to wait for given message
  //  Arguments:
  //    Message m - Message to wait for
  //  Return - boolean success
  bool waitFor(Message m){
    while(true)
      if(m.equal(messHandl.receiveMessageBlocking()))
        return true;
  }
  
  //Function to test communications between Arduino MEGA and Raspberry Pi
  //  Return - int holding execution mode
  int commTest(){
    Message m;
    while(true){
      messHandl.sendMessage('p', "T");
      delay(2);
      m = messHandl.receiveMessage();
      if(!m.isNull())
        if(m.source == 'p')
          return m.message.toInt();
      delay(20);
    }
  }

  //Waits for and communicates with drones
  //  Arguments:
  //    int i - Drone index number
  //  Return - Drone d
  Drone findDrone(int i){
    this->waitFor(Message('?', 'a', "C"));
    messHandl.sendMessage('0' + i, "C");
    while(true){
      Message m = messHandl.receiveMessageBlocking();
      if(m.source == ('0' + i))
        return Drone(i, m.message[0]);
    }
  }

  //Sends Drone info to the raspberry pi
  //  Arguments:
  //    vector<Drone> ds - Vector containing drones to send
  //  Return - boolean success
  bool sendDrones(vector<Drone> ds){
    for(vector<Drone>::iterator it = ds.begin(); it != ds.end(); ++it){
      messHandl.sendMessage('p', String("" + it->getNum() + it->getID()));
      this->waitFor(Message('p', 'a', "A"));
    }
  }

  //Connects to all Drones
  //  Arguments:
  //    int num - Number of drones to connect
  //  Return - vector<Drone> - Vector containing all Drone objects
  vector<Drone> findDrones(int num) {
    vector<Drone> tempDrones;
    for(int i = 1; i <= num; i++)
      tempDrones.push_back(this->findDrone(i));
    return tempDrones;
  }
};




//Global Variables
int MODE;           //Execution mode, received from Pi
SerialCom SER;
vector<Drone> drones;
Message mess;
Servo TowerRaise, TowerRotate;


//Helper and Wrapper Functions

//Waits for high signal from Start button
void waitForStart(){
  while(digitalRead(SignalPin) != HIGH){
    delay(10);
  }
}

//Wrapper to begin rotation
void setRotation(int speed){
  TowerRotate.writeMicroseconds(ServoMid + speed);
}

//Wrapper to raise camera arm
void raiseTower(){
  for(int i = TowerLoweredDegree; i >= TowerRaisedDegree; i--){
    TowerRaise.write(i);
    delay(5);
  }
}

void lowerTower(){
  for(int i = TowerRaisedDegree; i <= TowerLoweredDegree; i++){
    TowerRaise.write(i);
    delay(5);
  }
}

//Actions to be taken at the end of execution
void finishingActions(){
  messHandl.sendMessage('A', "B");          //Send Stop signal
  setRotation(0);                           //Stop Rotation
  lowerTower();                             //Lower Tower
  digitalWrite(FinishingLightPin, HIGH);    //Activate Finishing Light
  while(true){
    delay(100);                             //Stop processing by looping indefinitely
  }
}

void startstoploop(){
  if(digitalRead(6) == HIGH)
    Serial.write("High");
  else
    Serial.write("Low");
}

void setup() {
  
  TowerRaise.write(TowerLoweredDegree);
  TowerRaise.attach(TowerRaiseServoPin);
  setRotation(0);
  TowerRotate.attach(TowerRotateServoPin);
  
  Serial.begin(9600);
  while(!Serial){}
  // put your setup code here, to run once:
  pinMode(FinishingLightPin, OUTPUT);
  digitalWrite(FinishingLightPin, HIGH);
  Serial.println("test");
  delay(3000);
  digitalWrite(FinishingLightPin, LOW);
  Serial.println("Pins");
  pinMode(7, OUTPUT);
  pinMode(6, INPUT);
  digitalWrite(7, HIGH);
  Serial.println("Rotation");
  waitForStart();
  delay(2000);
  setRotation(0);
  delay(2000);
  setRotation(30);
  raiseTower();
  delay(15000);
  setRotation(0);
  lowerTower();
  digitalWrite(FinishingLightPin, HIGH);
}



void loop() {
  // put your main code here, to run repeatedly:
  startstoploop();
}
