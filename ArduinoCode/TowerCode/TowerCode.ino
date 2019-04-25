#include <ArduinoSTL.h>
#include <Servo.h>

using namespace std;


//Global Constants
#define SignalPin 6             //Input pin that receives signal from Start and Stop buttons
#define OutputHigh 7
#define FinishingLightPin 4      //Finishing Light Pin
#define TowerRaiseServoPin 3     //Tower Raise/Lower Servo Pin
#define TowerRotateServoPin 2    //Tower Rotation Servo Pin
#define TowerRaisedDegree 117     //Degree at which Camera Arm is raised
#define TowerLoweredDegree 180   //Degree at which Camera Arm is lowered
#define RotationSpeed 30         //Degree speed at which Tower Rotation Servo rotates
#define ServoMid 1636
const byte checksumConst = 0x01 + 0x01 + (byte)'s' + (byte)'a' + (byte)'x';
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
    this->machineID = '0';
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

vector<Drone> drones;


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
    Message receiveAnyMessageBlocking(){        //TODO Fix for XBee comms
      while(true){
        while(Serial.available() > 0)
          if(Serial.read() == 's'){
            String input = Serial.readStringUntil('x');
            Message output = Message(input[0], input[1], input.substring(2));
            return output;
          }
        while(Serial1.available() > 0)
          if(Serial1.read() == 0x7E){
            byte temp[7];
            Serial1.readBytes(temp, 7);
            String input = Serial1.readStringUntil('x');
            Serial1.readBytes(temp, 1);
            Message output = Message(input[1], input[2], input.substring(3));
            return output;
          }
        delay(10);
      }
    }

    //Receives a message while blocking execution
    //  Returns - Message - message received
    Message receiveMessageBlocking(){         //TODO Fix for XBee comms
      while(true){
        while(Serial.available() > 0)
          if(Serial.read() == 's'){
            String input = Serial.readStringUntil('x');
            if(input[1] == 'a' || input[1] == 'A'){
              Message output = Message(input[0], input[1], input.substring(2));
              return output;
            }
          }
        while(Serial1.available() > 0){
          if(Serial1.read() == 0x7E){
            byte  temp[7];
            Serial1.readBytes(temp, 7);
            String input = Serial1.readStringUntil('x');
            Serial1.readBytes(temp, 1);
            if(input[2] == 'a' || input[2] == 'A'){
              Message output = Message(input[1], input[2], input.substring(3));
              return output;
            }
          }
        }
        delay(10);
      }
    }

    //Receive message from XBee, helper for receiveMessage()
    // Returns:
    //    Message message or empty message
    Message receiveXBeeMessage(){
      if(Serial1.available() > 0) {
        while(Serial1.read() != 0x7E)
          if(Serial1.available() == 0)
            return Message();
        byte  temp[7];
        Serial1.readBytes(temp, 7);
        String input = Serial1.readStringUntil('x');
        Serial1.readBytes(temp, 1);
        if(input[2] == 'a' || input[2] == 'A'){
          Message output = Message(input[1], input[2], input.substring(3));
          return output;
        }
      }
      return Message();
    }

    //Receives message without blocking execution
    //  Return - Message or NULL
    Message receiveMessage(){             //TODO Fix for XBee comms
      if(Serial.available() > 0) {
        while(Serial.read() != 's')
          if(Serial.available() == 0)
            return receiveXBeeMessage();
        String input = Serial.readStringUntil('x');
        if(input[1] == 'a' || input[1] == 'A'){
          Message output = Message(input[0], input[1], input.substring(2));
          return output;
        }
      }
      return receiveXBeeMessage();
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
      this->serialString(String("sa" + String(d) + m + "x"));
    }

    //Receives binary image packet
    //  Arguments:
    //    byte[] outputBuffer
    //    int size - Size of packet
    void receiveImagePacket(byte* outputBuffer, int size){
      byte ignore[7];
      while(Serial1.read() != 0x7E) {}
      Serial1.readBytes(ignore, 7);
      for(int i = 0; i < size; i++)
        outputBuffer[i] = Serial1.read();
      Serial1.readBytes(ignore, 1);
    }

    //Generates XBee frame and sends message
    //  Arguments:
    //    char d - Destination character
    //    String m - Message
    bool sendXBeeMessage(char d, String m){
      int length = m.length() + 9;
      byte lengthLow = (byte) length;
      byte lengthHigh = (byte) (length >> 8);
      int dest = '0';
      for(vector<Drone>::iterator it = drones.begin(); it != drones.end(); ++it)
        if(it->getNumChar() == d)
          dest = it->getID();
      byte destHigh, destLow, destChecksum;
      switch(dest){
        case '1':
          destHigh = 0x00;
          destLow = 0x01;
          destChecksum = destLow;
          break;
        case '2':
          destHigh = 0x00;
          destLow = 0x02;
          destChecksum = destLow;
          break;
        case '3':
          destHigh = 0x00;
          destLow = 0x03;
          destChecksum = destLow;
          break;
        case '4':
          destHigh = 0x00;
          destLow = 0x04;
          destChecksum = destLow;
          break;
        default:
          destHigh = 0xFF;
          destLow = 0xFF;
          destChecksum = 0xFF + 0xFF;
          break;
      }
      Serial1.write(0x7E);
      Serial1.write(lengthHigh);
      Serial1.write(lengthLow);
      Serial1.write(0x01);
      Serial1.write(0x00);
      Serial1.write(destHigh);
      Serial1.write(destLow);
      Serial1.write(0x01);
      Serial1.write((byte)'s');
      Serial1.write((byte)'a');
      Serial1.write((byte)d);
      byte checksum = checksumConst + destChecksum + (byte)d;
      for(int i = 0; i < m.length(); i++){
        byte temp = (byte)m[i];
        checksum += temp;
        Serial1.write(temp);
      }
      Serial1.write((byte)'x');
      Serial1.write((byte)(0xFF - checksum));
    }

    //Writes instruction to XBee
    //  Arguments:
    //    byte[] instr - Instruction to send - 8 bytes
    void writeInstruction(byte instr[8], byte destHigh, byte destLow, byte destChecksum){
      Serial1.write(0x7E);
      Serial1.write(0x00);
      Serial1.write(0x0D);
      Serial1.write(0x01);
      Serial1.write(0x00);
      Serial1.write(destHigh);
      Serial1.write(destLow);
      Serial1.write(0x01);
      byte checksum = 0x02 + destChecksum;
      for(int i = 0; i < 8; i++){
        byte temp = instr[i];
        checksum += temp;
        Serial1.write(temp);
      }
      Serial1.write((byte)(0xFF - checksum));
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
  //    char d - Destination character
  void transmitInstruction(char d){            //TODO Fix for XBee comms
    messHandl.sendXBeeMessage(d, "I");
    byte temp[8];
    int dest = '0';
    for(vector<Drone>::iterator it = drones.begin(); it != drones.end(); ++it)
      if(it->getNumChar() == d)
        dest = it->getID();
    byte destHigh, destLow, destChecksum;
    switch(dest){
      case '1':
        destHigh = 0x00;
        destLow = 0x01;
        destChecksum = destLow;
        break;
      case '2':
        destHigh = 0x00;
        destLow = 0x02;
        destChecksum = destLow;
        break;
      case '3':
        destHigh = 0x00;
        destLow = 0x03;
        destChecksum = destLow;
        break;
      case '4':
        destHigh = 0x00;
        destLow = 0x04;
        destChecksum = destLow;
        break;
      default:
        destHigh = 0xFF;
        destLow = 0xFF;
        destChecksum = 0xFF + 0xFF;
        break;
    }
    while(true){
      Serial.readBytes(temp, 8);
      messHandl.writeInstruction(temp, destHigh, destLow, destChecksum);
      if((char) temp[7] == 'E')
        break;
    }
  }

  //Function to transmit a binary image file in chunks of 32 bytes
  //  Arguments:
  //    int size - Size of file in bytes
  void transmitBinaryImage(int size){               //TODO Fix for XBee comms
    messHandl.sendMessage('p', "L");
    messHandl.sendMessage('p', String(size));
    int remainder = size % 256;
    byte temp[256];
    byte ignore[7];
    for(int i = 0; i < size; i += 256){
      while(Serial1.read() != 0x7E) {}
      Serial1.readBytes(ignore, 7);
      Serial1.readBytes(temp, 256);
      Serial1.readBytes(ignore, 1);
      Serial.write(temp, 256);
    }
    if(remainder > 0){
      while(Serial1.read() != 0x7E) {}
      Serial1.readBytes(ignore, 7);
      Serial1.readBytes(temp, remainder);
      Serial1.readBytes(ignore, 1);
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
    messHandl.sendMessage('p', "T");
    while(true){
      digitalWrite(FinishingLightPin, HIGH);
      delay(100);
      digitalWrite(FinishingLightPin, LOW);
      m = messHandl.receiveMessage();
      if(m.source == 'p')
        return (m.message[0] - '0');
      delay(100);
    }
  }

  //Waits for and communicates with drones
  //  Arguments:
  //    int i - Drone index number
  //  Return - Drone d
  Drone findDrone(int i){
    this->waitFor(Message('?', 'a', "C"));
    messHandl.sendXBeeMessage('0' + i, "C");
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
      messHandl.sendMessage('p', String(it->getNum()) + String(it->getID()));
      this->waitFor(Message('p', 'a', "A"));
    }
  }

  //Connects to all Drones
  //  Arguments:
  //    int num - Number of drones to connect
  //  Return - vector<Drone> - Vector containing all Drone objects
  vector<Drone> findDrones(int num) {
    vector<Drone> tempDrones;
    for(int i = 1; i <= num; i++){
      digitalWrite(FinishingLightPin, HIGH);
      tempDrones.push_back(this->findDrone(i));
      digitalWrite(FinishingLightPin, LOW);
      delay(2000);
      for(int j = 0; j < i; j++){
        digitalWrite(FinishingLightPin, HIGH);
        delay(500);
        digitalWrite(FinishingLightPin, LOW);
        delay(500);
      }
    }
    return tempDrones;
  }
};




//Global Variables
int MODE;           //Execution mode, received from Pi
SerialCom SER;
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
  //TODO Send to drones separately
  messHandl.sendMessage('d', "B");
  messHandl.sendMessage('p', "B");          //Send Stop signal
  setRotation(0);                           //Stop Rotation
  lowerTower();                             //Lower Tower
  digitalWrite(FinishingLightPin, HIGH);    //Activate Finishing Light
  while(true){
    delay(100);                             //Stop processing by looping indefinitely
  }
}


void setup() {
  //Begin Serial
  Serial.begin(9600);
  while(!Serial){}
  Serial1.begin(9600);
  while(!Serial1){}
  SER = SerialCom();

  pinMode(FinishingLightPin, OUTPUT);
  digitalWrite(FinishingLightPin, HIGH);
  pinMode(SignalPin, INPUT);
  pinMode(OutputHigh, OUTPUT);
  digitalWrite(OutputHigh, HIGH);

  //Instantiate and reset servos
  TowerRaise.write(TowerLoweredDegree);
  TowerRaise.attach(TowerRaiseServoPin);
  setRotation(0);
  TowerRotate.attach(TowerRotateServoPin);

  //Establish communication with Raspberry Pi
  MODE = 3;
  MODE = SER.commTest();
  
  //Wait for and communicate with number of drones based on execution mode
  switch(MODE){
  case 1:
    drones = SER.findDrones(4);
    SER.sendDrones(drones);
    break;
  case 2:
    drones = SER.findDrones(2);
    SER.sendDrones(drones);
    break;
  case 3:
    break;
  case 4:
    drones = SER.findDrones(1);
    SER.sendDrones(drones);
    break;
  case 5:
    break;
  }

  //Wait for start signal
  digitalWrite(FinishingLightPin, HIGH);
  waitForStart();
  digitalWrite(FinishingLightPin, LOW);
  delay(5000);          //MANDATORY 5 SECOND DELAY AFTER START

  //Begin Execution
  if(MODE == 1 || MODE == 2 || MODE == 3 || MODE == 5){
    messHandl.sendXBeeMessage('d', "S");
    setRotation(RotationSpeed);
    raiseTower();
  }
  messHandl.sendMessage('p', "S");
}

void loop() {
  //Handle communications
  mess = messHandl.receiveMessage();
  if(!mess.isNull()){
    //Handle control messages first
    switch(mess.message[0]){
      case 'B':   //Stop signal from Raspberry Pi
        finishingActions();
        break;
      case 'I':  //Send instructions to Drone
        SER.transmitInstruction(mess.message[1]);
        break;
      case 'L':   //Receive and transmit image from Camera Drone
        mess = messHandl.receiveMessageBlocking();
        SER.transmitBinaryImage(mess.message.toInt());
        break;
    }
  }

  //Check if Stop button is pressed
  if(digitalRead(SignalPin) == LOW)
    finishingActions();
}
