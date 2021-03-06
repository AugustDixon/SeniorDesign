#include <ArduinoSTL.h>
#include <Servo.h>
#define MachineID '2'               //Hardware ID for each drone. Will be changed for each and written on chassis
#if MachineID == '1'
  #include <ArduCAM.h>
  #include <SPI.h>
  #include "memorysaver.h"
#endif

using namespace std;


//Global Constants
#define CSPin 7                      //ArduCAM Pin
#define FinishingLightPin 4          //Finishing Light Pin
#define LServoPin 5                  //Left Wheel Servo Pin 
#define RServoPin 3                  //Right Wheel Servo Pin 
#define ClampStrokePin 9             //Clamp Stroke Control Servo Pin 
#define ClampRaisePin 6              //Clamp Raise/Lower Servo Pin 
#define ClampStrokeOpenDegree 90     //Degree at which clamp is open 
#define ClampStrokeCloseDegree 0     //Degree at which clamp is closed 
#define ClampRaiseDegree 165         //Degree at which clamp is raised 
#define ClampLowerDegree 90          //Degree at which clamp is lowered 
#define ClampRaiseLowerTime 1000     //Over-Estimate Time required to raise/lower clamp (ms) //TODO
#define ClampOpenCloseTime 1000      //Over-Estimate Time required to open/cloase clamp (ms) //TODO
#define TurnSpeed 40                 //Constant Turn speed
#define BlockLiftTime 1000           //Over-Estimate time required to close/lift block (ms) //TODO
#define BlockLowerTime 1000          //Over-Estimate time required to lower/open block (ms) //TODO
#define ShortForwardTime 1000        //Over-Estimate time required to perform a constant short forward movement (ms) //TODO
#define ShortBackwardTime 1000       //Over-Estimate time required to perform a constant short backward movement (ms) //TODO
#define CaptureTime 1000             //Over-Estimate time required to capture and transmit an image (ms) //TODO
#define OpeningTime 1000             //Specific Time required for opening move forward (ms) //TODO
#define ServoMid 1500

#if MachineID == '1'
  #define ServoLOffset 0
  #define ServoROffset 0
#endif
#if MachineID == '2'
  #define ServoLOffset 0
  #define ServoROffset 0
#endif
#if MachineID == '3'
  #define ServoLOffset 0
  #define ServoROffset 0
#endif
#if MachineID == '4'
  #define ServoLOffset 0
  #define ServoROffset 0
#endif




const byte checksumConst = 0x02 + (byte)'s' + (byte)'a' + (byte)'x';



char droneChar = '?';               //Software ID of drone, given when connected to Arduino
Servo LServo, RServo, ClampStrokeServo, ClampRaiseServo;
#if MachineID == '1'
  ArduCAM myCAM(OV2640, CSPin);
#endif

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
      Message(droneChar, d, m);
    }
    Message(){
      this->source = ' ';
      this->dest = ' ';
      this->message = "";
    }
    ~Message(){  }
    
    //Receives a message regardless of destination while blocking execution
    //  Returns - Message - message received
    Message receiveAnyMessageBlocking(){
      while(true){
        while(Serial.available() > 0)
          if(Serial.read() == 0x7E){
            for(int i = 0; i < 7; i++)
              Serial.read();
            String input = Serial.readStringUntil('x');
            Serial.read();
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
          if(Serial.read() == 0x7E){
            for(int i = 0; i < 7; i++)
              Serial.read();
            String input = Serial.readStringUntil('x');
            Serial.read();
            if(input[1] == 'd' || input[1] == 'A' || input[1] == droneChar){
              Message output = Message(input[0], input[1], input.substring(2, input.length() - 3));
              return output;
            }
          }
        delay(20);
      }
    }

    //Receives and ignores intruction transmission
    void ignoreInstructions(){
      byte temp[8];
      while(true){
        for(int i = 0; i < 9; i++)
          Serial.read();
        Serial.readBytes(temp, 8);
        Serial.read();
        if(((char) temp[7]) == 'E')
          break;
      }
    }

    //Receives and ignores binary image file
    void ignoreBinary(int size){
      int remainder = size % 256;
      for(int i = 0; i < size; i += 256)
        for(int j = 0; j < 265; j++)
          Serial.read();
      if(remainder > 0)
        for(int i = 0; i < 9 + remainder; i++)
          Serial.read();
    }
    
    //Receives message without blocking execution
    //Manages ignoring image files and other drone's instructions
    //  Return - Message or NULL
    Message receiveMessage(){
      if(Serial.available() > 0) {
        while(Serial.read() != 0x7E)
          if(Serial.available() == 0)
            return Message();
        for(int i = 0; i < 7; i++)
          Serial.read();
        String input = Serial.readStringUntil('x');
        Serial.read();
        if(input[1] == 'd' || input[1] == 'A' || input[1] == droneChar){
          Message output = Message(input[0], input[1], input.substring(2, input.length() - 3));
          return output;
        }
        else if(input[2] == 'I')
          this->ignoreInstructions();
        else if(input[2] == 'L'){
          Message temp = this->receiveAnyMessageBlocking();
          this->ignoreBinary(temp.message.toInt());
        }
      }
    }

    //Returns boolean of message equality
    //UNUSED IN DRONE CODE
    /*
    bool equal(Message b){
      if(this->source == b.source && this->dest == b.dest && this->message == b.message)
        return true;
      else
        return false;
    }
    */

    //Sends a message through Serial
    //  Arguments:
    //    char d - Destination character
    //    String m - Message
    bool sendMessage(char d, String m){
      int length = m.length() + 9;
      byte lengthHigh = (byte)(length >> 8);
      byte lengthLow = (byte)length;
      byte frame[9 + length] = {0x7E, lengthHigh, lengthLow, 0x01, 0x01, 0x00, 0x00, 0x00, (byte)'s', (byte)droneChar, (byte)'a'};
      byte checksum = checksumConst + (byte)droneChar;
      for(int i = 0; i < m.length(); i++){
        byte temp = (byte)m[i];
        checksum += temp;
        frame[11 + i] = temp;
      }
      frame[length + 7] = (byte)'x';
      frame[length + 8] = 0xFF - checksum;
      Serial.write(frame, length + 9);
      Serial.flush();
    }

    //Reads instruction from XBee and place in outputBuffer
    //  Arguments:
    //    byte[8] outputBuffer
    void receiveInstruction(byte outputBuffer[8]){
      for(int i = 0; i < 9; i++)
        Serial.read();
      for(int i = 0; i < 8; i++)
        outputBuffer[i] = Serial.read();
      Serial.read();
    }

    //Returns boolean comparison to NULL
    bool isNull(){
      return (this == NULL) || (this->source == ' ' && this->dest == ' ' && this->message == "");
    }
};

Message messHandl;


#if MachineID == '1'
  //Class for wrapping and managing camera functions
  class Camera{
  public:
    //Constructor handles instantiations and setup
    Camera(){  }
    ~Camera(){  }

    //Captures image from camera and sends image
    void captureAndSend(){
      myCAM.flush_fifo();
      myCAM.clear_fifo_flag();
      myCAM.start_capture();
      while(!(myCAM.read_reg(ARDUCHIP_TRIG) & CAP_DONE_MASK)) {} //Wait for capture to finish
      uint32_t length = myCAM.read_fifo_length();
      messHandl.sendMessage('a', String(length));
      int remainder = length % 256;

      byte checksum;
  
      byte frame[265] = {0x7E, 0x01, 0x05, 0x01, 0x01, 0x00, 0x00, 0x00};
      
      for(int i = 0; i < length; i += 256){
        checksum = 0x02;
        for(int j = 0; j < 256; j++){
          byte temp = (byte)myCAM.read_fifo();
          frame[8 + j] = temp;
          checksum += temp;
        }
        frame[264] = (byte)(0xFF - checksum);
        Serial.write(frame, 265);
      }
      length = remainder + 5;
      frame[1] = (byte)(length >> 8);
      frame[2] = (byte)length;
      checksum = 0x02;
      for(int i = 0; i < remainder; i++){
        byte temp = (byte)myCAM.read_fifo();
        frame[8 + i] = temp;
        checksum += temp;
      }
      frame[8 + remainder] = (byte)(0xFF - checksum);
      Serial.write(frame, remainder);
      
        
      myCAM.clear_fifo_flag();
    }
  };

  Camera CAM;
#endif





//Class for wrapping and managing movement functions
class Movement{
public:
  Movement(){  }
  ~Movement(){  }

  //Stops all wheel servos
  void stopMovement(){
    LServo.writeMicroseconds(ServoMid);
    RServo.writeMicroseconds(ServoMid);
  }

  //Sets all wheel servos to the same degree
  //  Arguments:
  //    int degree - Speed of movement - range (-90 - 90)
  void setMovement(int degree){
    int posTemp = ServoMid + degree + ServoLOffset;
    int negTemp = ServoMid - degree - ServoROffset;
    LServo.writeMicroseconds(posTemp);
    RServo.writeMicroseconds(negTemp);
  }

  //Sets wheels to turning
  //  Arguments:
  //    int direct - Direction of turning - (direct > 0) CW, (direct <= 0) CCW
  void setTurning(int direct){
    int speedL, speedR;
    if(direct > 0){
      speedL = ServoMid + TurnSpeed + ServoLOffset;
      speedR = ServoMid + TurnSpeed + ServoROffset;
    }
    else{
      speedL = ServoMid - TurnSpeed - ServoLOffset;
      speedR = ServoMid - TurnSpeed - ServoLOffset;
    }
    LServo.writeMicroseconds(speedL);
    RServo.writeMicroseconds(speedR);
  }

  //Lift block sequence
  void liftBlock(){
    ClampStrokeServo.write(ClampStrokeOpenDegree);
    for(int i = ClampRaiseDegree; i >= ClampLowerDegree; i--){
      ClampRaiseServo.write(i);
      delay(6);
    }
    delay(500);
    ClampStrokeServo.write(ClampStrokeCloseDegree);
    delay(1000);
    for(int i = ClampLowerDegree; i <= ClampRaiseDegree; i++){
      ClampRaiseServo.write(i);
      delay(6);
    }
  }

  //Deposits block
  void depositBlock(){
    for(int i = ClampRaiseDegree; i >= ClampLowerDegree; i--){
      ClampRaiseServo.write(i);
      delay(6);
    }
    ClampStrokeServo.write(ClampStrokeOpenDegree);
    delay(1000);
    for(int i = ClampLowerDegree; i <= ClampRaiseDegree; i++){
      ClampRaiseServo.write(i);
      delay(6);
    }
  }

  //Resets all Servos to default state
  void reset(){
    stopMovement();
    ClampStrokeServo.write(ClampStrokeOpenDegree);
    ClampRaiseServo.write(ClampRaiseDegree);
    delay(1000);
  }
};

Movement movement;


//Contains individual instructions and their operations
class Instruction{
private:
  char type;      //Opcode char for instruction
  int arg1;
  long arg2;
public:
  Instruction(){
    this->type = ' ';
    this->arg1 = 0;
    this->arg2 = 0;
  }
  Instruction(char t, int a1, long a2){
    this->type = t;
    this->arg1 = a1;
    this->arg2 = a2;
  }
  ~Instruction(){  }

  long getArg2(){  return this->arg2; }

  //Executes instruction
  void execute(){
    switch(this->type){
      case 'M': //Movement
        movement.setMovement(this->arg1);
        break;
      case 'T': //Turning
        movement.setTurning(this->arg1);
        break;
      case 'C': //Camera Capture
        #if MachineID == '1'
          messHandl.sendMessage('a', "L");
          CAM.captureAndSend();
        #endif
        break;
      case 'L': //Lift Block
        movement.liftBlock();
        break;
      case 'D': //Deposit Block
        movement.depositBlock();
        break;
      case 'F': //Activate Finishing Light
        digitalWrite(FinishingLightPin, HIGH);
        break;
    }
  }
};

//Contains and manages an entire instruction queue
class InstructionSet{
private:
  vector<Instruction> queue;
  unsigned long nextTime;       //Time the next instruction will start
  //Constant Instructions used in generation
  Instruction BlockLift, BlockDeposit, ShortForward, ShortBackward, Capture;
  
  //Convert two bytes to an int
  int bytesToInt(byte b0, byte b1){ 
    int val = 0;
    val += b0 << 8;
    val += b1;
    return val;
  }

  //Convert four bytes to a long
  long bytesToLong(byte b0, byte b1, byte b2, byte b3){
    long val = 0;
    val += b0 << 24;
    val += b1 << 16;
    val += b2 << 8;
    val += b3;
    return val;
  }

public:
  //Constructor instantiates building block instructions
  InstructionSet(){
    this->queue;
    this->BlockLift = Instruction('L', 0, BlockLiftTime);
    this->BlockDeposit = Instruction('D', 0, BlockLowerTime);
    this->ShortForward = Instruction('M', 100, ShortForwardTime);
    this->ShortBackward = Instruction('M', 80, ShortBackwardTime);
    this->Capture = Instruction('C', 0, CaptureTime);
    this->nextTime = 0;
  }
  ~InstructionSet(){ }

  //Clears queue and stops movement
  void clear(){
    this->queue.clear();
    movement.stopMovement();
  }

  void push(Instruction inst){
    this->queue.push_back(inst);
  }

  //Pop the next instruction and execute
  void executeNext(){
    if(!this->queue.empty()){
      Serial.write("Execute Next");
      movement.stopMovement();
      Instruction temp = this->queue.front();
      this->queue.erase(queue.begin());
      temp.execute();
      this->nextTime = millis() + temp.getArg2();
    }
  }

  //Receive instructions from Arduino and build queue
  void receiveInstructions(){
    byte temp[8];
    int tempI;
    long tempL;
    while(true){
      messHandl.receiveInstruction(temp);
      tempI = bytesToInt(temp[1], temp[2]);
      tempL = bytesToLong(temp[3], temp[4], temp[5], temp[6]);
      switch((char) temp[0]){
        case 'L':
          this->queue.push_back(this->BlockLift);
          break;
        case 'D':
          this->queue.push_back(this->BlockDeposit);
          break;
        case 'C':
          this->queue.push_back(this->Capture);
          break;
        case 'B':
          this->clear();
          break;
        default:
          this->queue.push_back(Instruction((char) temp[0], tempI, tempL));
          break;
      }
      if((char) temp[7] == 'E')
        break;
    }
    this->executeNext();
  }

  //Poll time to check for next instruction execution window
  void checkTime(){
    if(!this->queue.empty()){
      if(millis() >= this->nextTime)
        this->executeNext();
    }
    else
      movement.stopMovement();
  }

  //Simple instruction queue for beginning of round
  void openingActions(){
    this->queue.push_back(Instruction('M', 100, OpeningTime));
  }
};


//Connect to Arduino and receive droneChar ID for communication
void connectToArduino(){
  messHandl.sendMessage('a', "C");
  digitalWrite(FinishingLightPin, LOW);
  Message message = messHandl.receiveAnyMessageBlocking();
  digitalWrite(FinishingLightPin, HIGH);
  droneChar = message.dest;
  messHandl.sendMessage('a', String("" + MachineID));
  digitalWrite(FinishingLightPin, LOW);
}



Message m;
InstructionSet instructions;




void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!Serial){  }                 //Wait for Serial to begin

  //Instantiate Camera if MachineID is 1 (CameraDrone)
  #if MachineID == '1'
    pinMode(CSPin, OUTPUT);
    SPI.begin();
    myCAM.write_reg(ARDUCHIP_MODE, 0x00);
    myCAM.set_format(JPEG);
    myCAM.InitCAM();
    myCAM.OV2640_set_JPEG_size(OV2640_160x120);
    delay(1000);
  #endif
  instructions = InstructionSet();
  
  LServo.attach(LServoPin);
  RServo.attach(RServoPin);
  ClampRaiseServo.attach(ClampRaisePin);
  ClampStrokeServo.attach(ClampStrokePin);
  movement = Movement();
  movement.reset();
  
  
  pinMode(FinishingLightPin, OUTPUT);
  digitalWrite(FinishingLightPin, LOW);
  //connectToArduino();
  
  //Put instructions here
  instructions.push(Instruction('M', 30, 4000));
  instructions.push(Instruction('M', 0, 2000));
  instructions.push(Instruction('M', -30, 4000));
  instructions.push(Instruction('M', 0, 2000));
  instructions.push(Instruction('M', 60, 4000));
  instructions.push(Instruction('M', 0, 2000));
  instructions.push(Instruction('M', -60, 4000));
  instructions.push(Instruction('M', 0, 2000));
  //instructions.push(Instruction('T', 1, 4000));
  //instructions.push(Instruction('M', 0, 2000));
  //instructions.push(Instruction('T', 0, 4000));
  //instructions.push(Instruction('M', 0, 2000));
  //instructions.push(Instruction('L', 0, 5000));
  //instructions.push(Instruction('D', 0, 5000));
  instructions.push(Instruction('F', 0, 0));
}

void loop() {
  // put your main code here, to run repeatedly:
  instructions.checkTime();
}
