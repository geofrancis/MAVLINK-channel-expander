#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates


#include "mavlink/common/mavlink.h"
#include "mavlink/common/mavlink_msg_rc_channels.h"
//UART Serial2(4, 5, 0, 0);
unsigned long previousMillis = 0;        // will store last time LED was updated
const long interval = 3000;           // interval at which to blink (milliseconds)



#include <NeoPixelConnect.h>
NeoPixelConnect p(16, 1, pio0, 0);


/*functions

  sound power       2  A0
  sound throttle    A  A1
  top lights        2  A2
  camera switcher   3  A3
  camera pan        A  A4

  smoke throttle    A  13
  smoker power      2  12
  underlights       2  11


  crane             A  A4
  tow winch         3  A5
  front winch       3  A6


*/
// used pins

#define sound           1
#define soundthrottle   2
#define lights          3
#define campan          4

#define camswitch       5
#define underlight      6
#define smoke           7
#define smokethrottle   8

#define fwinch          9
#define twinch          10
#define crane           11


int ch1  = 0;
int ch2  = 0;
int ch3  = 0;
int ch4  = 0;
int ch5  = 0;
int ch6  = 0;
int ch7  = 0;
int ch8  = 0;
int ch9  = 0;
int ch10 = 0;
int ch11 = 0;
int ch12 = 0;
int ch13 = 0;
int ch14 = 0;
int ch15 = 0;
int ch16 = 0;

#define soundbusy        0
int soundon = 0;

#define PWMHigh 2000
#define PWMMid  1500
#define PWMLow  1000



int FUNCTION = 0 ;
int functionselect = 0;
int THRO = 0;
int THRM = 0;
int smthrottlepwm;
int sothrottlepwm;
int cranepwm;

uint16_t channels[16];
bool failSafe;
bool lostFrame;


const int ledPin =  LED_BUILTIN;// the number of the LED pin
int ledState = LOW;


void setup()
{

  Serial.begin (115200); //The SBUS is a non standard baud rate which can be confirmed using an oscilloscope
  //  SerialS.begin(9600);
  Serial.println("START");

  Serial1.setRX(1);
  Serial1.setTX(0);
  Serial1.begin(115200);

  Serial2.setRX(5);
  Serial2.setTX(4);
  Serial2.begin(115200);


  Wire.setSDA(8);
  Wire.setSCL(9);
  Wire.setClock(400000); // use 400 kHz I2C
  Wire.begin();


  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  p.neoPixelFill(255, 0, 0, true);

  pinMode (soundbusy, INPUT);
  pinMode(ledPin, OUTPUT);

  //set startup positions
  pwm.writeMicroseconds(sound, PWMLow);
  pwm.writeMicroseconds(soundthrottle, PWMMid);
  pwm.writeMicroseconds(lights, PWMLow);
  pwm.writeMicroseconds(smoke, PWMLow);
  pwm.writeMicroseconds(smokethrottle, PWMLow);
  pwm.writeMicroseconds(crane, PWMMid);
  pwm.writeMicroseconds(fwinch, PWMMid);
  pwm.writeMicroseconds(twinch, PWMMid);
  pwm.writeMicroseconds(underlight, PWMLow);
  pwm.writeMicroseconds(campan, PWMMid);
  pwm.writeMicroseconds(camswitch, PWMLow);
  pwm.writeMicroseconds(soundthrottle, PWMMid);


  digitalWrite(ledPin, LOW);

  request_datastream();



}







void loop() {
  MavLink_receive();
  Serial.println(ch7);
  FUNCTION =  (ch7);

  if (FUNCTION <= 990) {
    pwm.writeMicroseconds(fwinch, PWMMid);
    pwm.writeMicroseconds(twinch, PWMMid);
    // Serial.println(channels[16]);

  }
  if (FUNCTION > 1045 && FUNCTION < 1055) {

    Serial.println("SMOKE OFF");
    pwm.writeMicroseconds(smoke, PWMLow);
    THRM = 0;
    //   Serial.println(channels[16]);
  }
  if (FUNCTION > 1070 && FUNCTION < 1080) {
    THRM = 1;
    Serial.println("SMOKE FULL");
    pwm.writeMicroseconds(smoke, PWMHigh);
    //  Serial.println(channels[16]);

  }
  if (FUNCTION > 1090 && FUNCTION < 1115) {
    THRM = 2;
    Serial.println("SMOKE THRM");
    pwm.writeMicroseconds(smoke, PWMHigh);
    //  Serial.println(channels[16]);

  }
  if (FUNCTION > 1165 && FUNCTION < 1180) {
    pwm.writeMicroseconds(sound, PWMLow);
    Serial.println("SOUND OFF");
    //   Serial.println(channels[16]);
  }
  if (FUNCTION > 1210 && FUNCTION < 1235) {
    pwm.writeMicroseconds(sound, PWMHigh);
    Serial.println("SOUND IDLE");
    THRO = 1;
    //  Serial.println(channels[16]);
  }
  if (FUNCTION > 1270 && FUNCTION < 1280) {
    pwm.writeMicroseconds(sound, PWMHigh);
    Serial.println("SOUND THRM");
    THRO = 2;
    //   Serial.println(channels[16]);
  }
  if (FUNCTION > 1320 && FUNCTION < 1330) {
    pwm.writeMicroseconds(lights, PWMLow);
    Serial.println("Lights OFF");
    //   Serial.println(channels[16]);
  }
  if (FUNCTION > 1370 && FUNCTION < 1380) {
    pwm.writeMicroseconds(lights, PWMHigh);
    Serial.println("Lights ON");
    //  Serial.println(channels[16]);
  }
  if (FUNCTION > 1520 && FUNCTION < 1540) {
    pwm.writeMicroseconds(fwinch, PWMHigh);
    Serial.println("Anchor Winch UP");
    //  Serial.println(channels[16]);
  }
  if (FUNCTION > 1570 && FUNCTION < 1590) {
    pwm.writeMicroseconds(fwinch, PWMLow);
    Serial.println("Anchor Winch down");
    //  Serial.println(channels[16]);
  }
  if (FUNCTION > 1420 && FUNCTION < 1440) {
    pwm.writeMicroseconds(twinch, PWMHigh);
    Serial.println("Tow Winch UP");
    //Serial.println(channels[16]);
  }
  if (FUNCTION > 1470 && FUNCTION < 1490) {
    pwm.writeMicroseconds(twinch, PWMLow);
    Serial.println("Tow Winch down");
    //    Serial.println(channels[16]);
  }


  soundon = digitalRead(soundbusy);

  if (soundon = 1) {

    if (FUNCTION > 1590 && FUNCTION < 1605) {
      Serial.println("HORN 1");
      Serial.println(channels[16]);
    }
    if (FUNCTION > 1605 && FUNCTION < 1615) {

      Serial.println("HORN 2");
      Serial.println(channels[16]);
    }
    if (FUNCTION > 1617 && FUNCTION < 1625) {

      Serial.println("HORN 3");
      Serial.println(channels[16]);
    }
    if (FUNCTION > 9999 && FUNCTION < 8888) {

      Serial.println("HORN 4");
      Serial.println(channels[16]);
    }
  }


  if (FUNCTION > 1725 && FUNCTION < 1745) {
    pwm.writeMicroseconds(underlight, PWMHigh);
    Serial.println("Underlights on");
    //  Serial.println(channels[16]);
  }
  if (FUNCTION > 1765 && FUNCTION < 1785) {
    pwm.writeMicroseconds(underlight, PWMLow);
    Serial.println("Underlights off");
    //    Serial.println(channels[16]);
  }

  else if (FUNCTION >= 2000) {
    functionselect = 11;
  }





  if     (THRM == 0) {
    pwm.writeMicroseconds(smokethrottle, PWMLow);
  }
  if     (THRM == 1) {
    pwm.writeMicroseconds(smokethrottle, PWMHigh);
  }
  if     (THRM == 2) {
    pwm.writeMicroseconds(smokethrottle, ch15);
  }
  if     (THRO == 1) {
    pwm.writeMicroseconds(soundthrottle, PWMMid);
  }
  if     (THRO == 2) {
    pwm.writeMicroseconds(soundthrottle, ch15);
  }

  pwm.writeMicroseconds(crane, ch14);
  pwm.writeMicroseconds(campan, ch13);

  //Serial.println(channels[16]);
  //Serial.println(channels[16]);


}



//function called by arduino to read any MAVlink messages sent by serial communication from flight controller to arduino
void MavLink_receive()
{
  mavlink_message_t msg;
  mavlink_status_t status;

  while (Serial2.available())
  {
    uint8_t c = Serial2.read();

    //Get new message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {
      p.neoPixelFill(0, 0, 255, true);
      //Handle new message from autopilot
      switch (msg.msgid)
      {

          /*    case MAVLINK_MSG_ID_GPS_RAW_INT:
                {
                  mavlink_gps_raw_int_t packet;
                  mavlink_msg_gps_raw_int_decode(&msg, &packet);

            mavlink_gps_raw_int_t datagps;
                  mavlink_msg_gps_raw_int_decode (&msg, &datagps);
                  //Serial.println("PX HB");
                 Serial.println("GPS Data ");
                  Serial.print("time usec: ");
                  Serial.println(datagps.time_usec);
                  Serial.print("lat: ");
                  Serial.println(datagps.lat);
                  Serial.print("lon: ");
                  Serial.println(datagps.lon);
                  Serial.print("alt: ");
                  Serial.println(datagps.alt);
                  Serial.print("Sattelite visible: ");
                  Serial.println(datagps.satellites_visible);
                  //Serial.println(datagps.eph);
                  //Serial.println(datagps.epv);
          */
          //  }
          break;
          /*
                  case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
                    {
                      //mavlink_message_t* msg;
                      mavlink_sys_status_t sys_status;
                      mavlink_msg_sys_status_decode(&msg, &sys_status);
                      Serial.print("PX SYS STATUS: ");
                      Serial.print("[Bat (V): ");
                      Serial.print(sys_status.voltage_battery);
                      Serial.print("], [Bat (A): ");
                      Serial.print(sys_status.current_battery);
                      Serial.print("], [Comms loss (%): ");
                      Serial.print(sys_status.drop_rate_comm);
                      Serial.println("]");
                    }

          */
          break;
          /*
                  case MAVLINK_MSG_ID_ATTITUDE:  // #30
                    {

                      mavlink_attitude_t attitude;
                      mavlink_msg_attitude_decode(&msg, &attitude);
                      Serial.println("PX ATTITUDE");
                      Serial.println(attitude.roll);
                      //if (attitude.roll > 1) leds_modo = 0;
                      //else if (attitude.roll < -1) leds_modo = 2;
                      //else leds_modo = 1;
                    }
          */
          break;
          /*
                  case MAVLINK_MSG_ID_RC_CHANNELS_RAW:  // #35
                    {
                      mavlink_rc_channels_raw_t chs;
                      mavlink_msg_rc_channels_raw_decode(&msg, &chs);
                      Serial.print("Chanel 6 (3-Kanal Schalter): ");
                      Serial.println(chs.chan6_raw);
                    }
          */
          break;
          /*
            case MAVLINK_MSG_ID_RC_CHANNELS_SCALED:  // #35
            {
            mavlink_rc_channels_scaled_t RCCHANNEL;
            mavlink_msg_rc_channels_scaled_decode(&msg, &RCCHANNEL);
            Serial.print("Chanel 6 (3-Kanal Schalter): ");
            int RAW_SERVO = RCCHANNEL.chan6_scaled;
            Serial.println(RAW_SERVO);
            Serial.print("Chanel 5 (Schub): ");
            Serial.println(RCCHANNEL.chan5_scaled);
            Serial.print("Drei Kanal: ");
            Serial.println(mavlink_msg_rc_channels_scaled_get_chan6_scaled(&msg));
            Serial.print("Schub: ");
            Serial.println(mavlink_msg_rc_channels_scaled_get_chan5_scaled(&msg));
            }
          */
          break;
          /*
            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:  // #30
            {
            mavlink_global_position_int_t Position;
            //mavlink_msg_attitude_decode(&msg, &attitude);
            mavlink_msg_global_position_int_decode(&msg, &Position);
          */
          break;

        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {

            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);

            Serial.print("\nFlight Mode: (10 ");
            Serial.println(hb.custom_mode);
            //  Serial.print("Type: ");
            //  Serial.println(hb.type);
            //  Serial.print("Autopilot: ");
            //  Serial.println(hb.autopilot);
            //   Serial.print("Base Mode: ");
            //   Serial.println(hb.base_mode);
            //   Serial.print("System Status: ");
            //   Serial.println(hb.system_status);
            //   Serial.print("Mavlink Version: ");
            //   Serial.println(hb.mavlink_version);
            //    Serial.println();
          }
          break;

        /*  case MAVLINK_MSG_ID_STATUSTEXT: //  #253  https://mavlink.io/en/messages/common.html#STATUSTEXT
            {
              mavlink_statustext_t packet;
              mavlink_msg_statustext_decode(&msg, &packet);
              Serial.print("=STATUSTEXT");
              Serial.print(" severity:");
              Serial.print(packet.severity);
              Serial.print(" text:");
              Serial.print(packet.text);
              break;
            }*/
        case MAVLINK_MSG_ID_RC_CHANNELS:  // #35
          {
            mavlink_rc_channels_t chs;
            mavlink_msg_rc_channels_decode(&msg, &chs);
            Serial.print("Chanel 1: ");
            Serial.println(chs.chan1_raw );
            ch1  = (chs.chan1_raw);
            ch2  = (chs.chan2_raw);
            ch3  = (chs.chan3_raw);
            ch4  = (chs.chan4_raw);
            ch5  = (chs.chan5_raw);
            ch6  = (chs.chan6_raw);
            ch7  = (chs.chan7_raw);
            ch8  = (chs.chan8_raw);
            ch9  = (chs.chan9_raw);
            ch10 = (chs.chan10_raw);
            ch11 = (chs.chan11_raw);
            ch12 = (chs.chan12_raw);
            ch13 = (chs.chan13_raw);
            ch14 = (chs.chan14_raw);
            ch15 = (chs.chan15_raw);
            ch16 = (chs.chan16_raw);
          }
          /*    case MAVLINK_MSG_ID_RC_CHANNELS_SCALED:  // #35
                {
                  mavlink_rc_channels_scaled_t RCCHANNEL;
                  mavlink_msg_rc_channels_scaled_decode(&msg, &RCCHANNEL);
                  Serial.print("Chanel 1 scaled: ");
                  int RAW_SERVO = RCCHANNEL.chan1_scaled;
                  Serial.println(RAW_SERVO);
                  Serial.print("Chanel 1 (raw): ");
                  Serial.println(RCCHANNEL.chan5_scaled);
                  Serial.print("Drei Kanal: ");
                  Serial.println(mavlink_msg_rc_channels_scaled_get_chan6_scaled(&msg));
                  Serial.print("Schub: ");
                  Serial.println(mavlink_msg_rc_channels_scaled_get_chan5_scaled(&msg));
                }*/
      }
    }
  }

}


//function called by arduino to read any MAVlink messages sent by serial communication from flight controller to arduino




void request_datastream() {
  //Request Data from Pixhawk
  uint8_t _system_id = 255; // id of computer which is sending the command (ground control software has id of 255)
  uint8_t _component_id = 2; // seems like it can be any # except the number of what Pixhawk sys_id is
  uint8_t _target_system = 1; // Id # of Pixhawk (should be 1)
  uint8_t _target_component = 0; // Target component, 0 = all (seems to work with 0 or 1
  uint8_t _req_stream_id = MAV_DATA_STREAM_ALL;
  uint16_t _req_message_rate = 0x08; //number of times per second to request the data in hex
  uint8_t _start_stop = 1; //1 = start, 0 = stop

  // STREAMS that can be requested
  /*
     Definitions are in common.h: enum MAV_DATA_STREAM and more importantly at:
     https://mavlink.io/en/messages/common.html#MAV_DATA_STREAM

     MAV_DATA_STREAM_ALL=0, // Enable all data streams
     MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
     MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
     MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
     MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
     MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
     MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot
     MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot
     MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot
     MAV_DATA_STREAM_ENUM_END=13,

     Data in PixHawk available in:
      - Battery, amperage and voltage (SYS_STATUS) in MAV_DATA_STREAM_EXTENDED_STATUS
      - Gyro info (IMU_SCALED) in MAV_DATA_STREAM_EXTRA1
  */

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_request_data_stream_pack(_system_id, _component_id, &msg, _target_system, _target_component, _req_stream_id, _req_message_rate, _start_stop);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)

  Serial2.write(buf, len); //Write data to serial port
}


/*
  void setmode_Auto() {
  //Set message variables
  uint8_t _system_id = 255; // id of computer which is sending the command (ground control software has id of 255)
  uint8_t _component_id = 2; // seems like it can be any # except the number of what Pixhawk sys_id is
  uint8_t _target_system = 1; // Id # of Pixhawk (should be 1)
  uint8_t _base_mode = 1;
  uint32_t _custom_mode = 10; //10 = auto mode

  /*
    Flight / Driving Modes (change custom mode above)
    0 - Manual
    1 - Acro
    3 - Steering
    4 - Hold
    5 - Loiter
    6 - Follow
    7 - Simple
    10  Auto
    11  RTL
    12  SmartRTL
    15  Guided


  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_set_mode_pack(_system_id, _component_id, &msg, _target_system, _base_mode, _custom_mode);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)
  Serial.print("\nsending set mode command...");
  Serial2.write(buf, len); //Write data to serial port
  }

*/


void sendtext() {


  uint8_t system_id = 1;
  uint8_t component_id = 2;
  uint8_t severity = 1;

  uint16_t id = 0;
  uint8_t chunk_seq = 0;



  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_statustext_pack(system_id, component_id, &msg, 0, "TEST", id, chunk_seq);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Serial2.write(buf, len);
}
