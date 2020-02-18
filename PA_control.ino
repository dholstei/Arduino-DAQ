#if 1 //---board/cpu stuff

#if defined(__AVR_ATmega2560__)
  #define MEGA
#elif defined(__SAM3X8E__)
  #define DUE
#endif

#ifdef MEGA
  #define IDENT "PAduino-Mega"
#elif defined(DUE)
  #define IDENT "PAduino-Due"
  #define DAC_OFFSET 66
#else
  #define IDENT "PAduino-RS485"
#endif

#define ANALOG_OFFSET 54
#endif //---end board/cpu stuff

#if 1 //---comm stuff
#define RS485_BUAD 19200
#define USB_BUAD 115200

#define WRITE_ADDRESS(STREAM) Serial1.begin(RS485_BUAD, SERIAL_8N2); Serial1.write(STREAM[0]); Serial1.end();
#define WRITE_STREAM(STREAM, I) pbyte = STREAM[I]; Serial1.begin(RS485_BUAD, (PARITY(pbyte)? SERIAL_8E1: SERIAL_8O1)); Serial1.write(STREAM[I]); Serial1.end();
#define READ_STREAM(BUF, S, I) Serial1.begin(RS485_BUAD, SERIAL_8N2); Serial1.readBytes(BUF + S, I); Serial1.end();
#define READ_STREAM_BEGIN(BUF, S, I) Serial1.begin(RS485_BUAD, SERIAL_8N2); Serial1.readBytes(BUF + S, I);
#define READ_STREAM_CPMLT(BUF, S, I) Serial1.readBytes(BUF + S, I); Serial1.end();

//---command defs
#define Set_PA_Status 0x1
#define Clear_PA_faults 0x2
#define Set_test_mode 0x3 //  not currently a thing. PA uses averaging over 1 second before sending data, I'd like to turn off averaging.
#define Get_PA_status 0x81
#define Get_all_data 0x82
#define Get_faults 0x83

#define MAX_BUF_LEN 100
#define MAX_DATA_SETS 51

unsigned char i, pbyte, pin, val=0; unsigned int val_analog;
unsigned char stream[15] = {0x0}, buf[MAX_DATA_SETS][MAX_BUF_LEN];
#ifdef SEND_FUNCTION
unsigned char dummy[] = {0x1, 0xFE, 0x1, 0x81, 0x47, 0x40, 0x0, 0x0, 0x0, 0x0};
#endif
unsigned char KA_Stream[10], KA_Buf[MAX_BUF_LEN];
char KA_buf_len = 0, KA_stream_len = 0, buf_len, stream_len;
long KA_dly = 0, KA_time = 0;
bool KeepAlived = false, busy = false, KA_Query = false;
unsigned char gpio_output = 0;  //  input=0, output=1, DAC=2
bool POL = false;	//	invert polarity, false is positive, true is negative
String cmd, key, value;
unsigned short crc, crc_recv, swp_dly;

//  pin assignments
#define RE 2
#define DE 3
#define RFMUTE_PIN 4
#define INTLK_PIN 5
#define BNC_PIN 20

#define PRINT_CHR(a) Serial.print("0x"); Serial.print(a, HEX); Serial.print("\t");
#define PARITY(x) ((~(x ^= (x ^= (x ^= x >> 4) >> 2) >> 1)) & 1) /* even parity */
#define WRITE_ENABLE digitalWrite(DE, HIGH); digitalWrite(RE, HIGH);
#define READ_ENABLE digitalWrite(DE, LOW); digitalWrite(RE, LOW);
#endif //---end comm stuff

void setup() {
  pinMode(DE, OUTPUT);  pinMode(RE, OUTPUT);
  pinMode(RFMUTE_PIN, OUTPUT);  pinMode(INTLK_PIN, OUTPUT);  pinMode(BNC_PIN, OUTPUT);
  Serial.begin(USB_BUAD); // opens Serial port, sets data rate to USB_BUAD bps
  Serial.setTimeout(20);
  //  Serial.println(IDENT);
  READ_ENABLE;
  digitalWrite(BNC_PIN, HIGH != POL);	//	XOR for logical
  swp_dly = 0;
}

void loop() {
  int i, k;
  
  if (!Serial.available()) {
    if (KeepAlived && (KA_time + KA_dly)>millis()) {KeepAlive(); KA_time = millis();}
    }
  else {
    cmd = Serial.readString(); cmd.toLowerCase();
    if (cmd.length() < 2) {return;}
    key = cmd; key.remove(cmd.indexOf(" "));
    crc = 0; k = 0;
    
    if (cmd.startsWith("*idn?\n")) { // identification
      Serial.println(IDENT);
    }
#ifdef SEND_FUNCTION
    else if (cmd.startsWith("send\n")) { // test using USB monitor
      WRITE_ENABLE;
      WRITE_ADDRESS(dummy);
      for (i=1; i<6; i++) {WRITE_STREAM(dummy, i);} 
      for (i=0; i<6; i++) {PRINT_CHR(dummy[i]);}; Serial.println("");
      READ_ENABLE;
      
      // read results
      READ_STREAM_BEGIN(buf[0], 0, 3); READ_STREAM_CPMLT(buf[0], 3, (buf[0][2] + 2)); KA_buf_len = buf[0][2] + 5;
      for (i=0; i<KA_buf_len; i++) {PRINT_CHR(buf[i]);}; Serial.println("");
    }
#endif
    else if (cmd.startsWith("query\n")) { // relay communications between USB and RS485
      Serial.print(":");
      Serial.readBytes(&stream_len, 1); Serial.readBytes(stream, stream_len);
      do {
        QueryIterate(0, false);
        crc = 1; // CalculateCRC16(buf[0], buf[0][2] + 3);
        if (++k > 4) break;
        } while (crc == 0); //  May change to calculate and compare CRC until correct
      Serial.write(buf_len); Serial.write(buf[0], buf_len);  // put on USB serial
    }
    else if (cmd.startsWith("sweep_delay")) { // set sweep delay
      cmd.remove(0, cmd.indexOf(" ")); cmd.trim();
      swp_dly = cmd.toInt();
      Serial.print("sweep_delay "); Serial.println(swp_dly);
    }
    else if (cmd.startsWith("trig:")) {
      cmd.remove(0, cmd.indexOf(":")+1); cmd.trim();
      POL = cmd.startsWith("neg");   //  POS or NEG, false is positive, true is negative
      Serial.print("TRIG:"); Serial.println(POL? "NEG" : "POS");
    }
#if defined(MEGA) || defined(DUE)
    else if (cmd.startsWith("gpio:")) { // GPIO device "gpio:_pin_no_:input/output:val"
      cmd.remove(0, cmd.indexOf(":")+1); cmd.trim(); pin = cmd.toInt();
      cmd.remove(0, cmd.indexOf(":")+1); cmd.trim(); gpio_output = cmd.startsWith("output");
      pinMode(pin, gpio_output ? OUTPUT : INPUT);
      if (gpio_output) {cmd.remove(0, cmd.indexOf(":")+1); cmd.trim(); val = cmd.toInt(); digitalWrite(pin, val);}
      else {val =  digitalRead(pin);}
      Serial.print("GPIO:"); Serial.print(pin); Serial.print(":"); Serial.print(gpio_output ? "OUTPUT:" : "INPUT:"); Serial.println(val);
    }
    else if (cmd.startsWith("analog:")) { // ANALOG device "analog:_pin_no_:input/output:val" (0-255 analog write)
      cmd.remove(0, cmd.indexOf(":")+1); cmd.trim(); pin = cmd.toInt();
      cmd.remove(0, cmd.indexOf(":")+1); cmd.trim(); analogWriteResolution(12);
      switch(cmd.charAt(0)) {
       case 'i' : /* input */
          pinMode(pin + ANALOG_OFFSET, INPUT); Serial.print("ANALOG:A"); Serial.print(pin); Serial.print(":INPUT:");
          val_analog =  analogRead(pin + ANALOG_OFFSET); Serial.println(val_analog);
          break;
       case 'o' : /* output */
          pinMode(pin, OUTPUT); Serial.print("ANALOG:"); Serial.print(pin); Serial.print(":OUTPUT:"); 
          cmd.remove(0, cmd.indexOf(":")+1); cmd.trim(); val = cmd.toInt(); analogWrite(pin, val); Serial.println(val);
          break;
#ifdef DUE
       case 'd' : /* DAC */
          pinMode(pin + DAC_OFFSET, OUTPUT); Serial.print("ANALOG:"); Serial.print(pin); Serial.print(":DAC:"); 
          cmd.remove(0, cmd.indexOf(":")+1); cmd.trim(); val_analog = cmd.toInt(); analogWrite(pin + DAC_OFFSET, val_analog); Serial.println(val_analog);
          break;
#endif
       default : /* mistake */
          Serial.print("ANALOG:"); Serial.print(pin);Serial.print(":");  Serial.print(cmd); Serial.print(":NOT SUPPORTED:"); Serial.println("NA");
      }
    }
#endif
    else if (cmd.startsWith("sweep")) { // sweep up to MAX_DATA_SETS points (key.equals("sweep"))
      cmd.remove(0, cmd.indexOf(" ")); cmd.trim();
      k = min(cmd.toInt(), MAX_DATA_SETS);
      Serial.print(":");
      Serial.readBytes(&buf_len, 1); Serial.readBytes(stream, buf_len);
      for (i=0; i<k; i++) {
        digitalWrite(BNC_PIN, LOW != POL);
        delay(swp_dly);
        QueryIterate(i, false);
      }
      Serial.print(":"); Serial.write(k);
      for (i=0; i<k; i++) {
        Serial.write(buf_len); Serial.write(buf[i], buf_len);  // put on USB serial
      }
    }
    else if (cmd.startsWith("cmd\n")) { // relay communications between USB and RS485, command only
      Serial.print(":");
      Serial.readBytes(&buf_len, 1);
      Serial.readBytes(stream, buf_len);
      WRITE_ENABLE;
      WRITE_ADDRESS(stream);
      for (i=1; i<buf_len; i++) {WRITE_STREAM(stream, i);}
      READ_ENABLE;
    }
    else if (cmd.startsWith("intlk")) { // set interlock
      cmd.remove(0, cmd.indexOf(" ")); cmd.trim();
      digitalWrite(INTLK_PIN, cmd.toInt()>0? HIGH : LOW);
      Serial.print(key); Serial.println(cmd.toInt()>0? " 1" : " 0");
    }
    else if (cmd.startsWith("rfmute")) { // set rf mute
      cmd.remove(0, cmd.indexOf(" ")); cmd.trim();
      digitalWrite(RFMUTE_PIN, cmd.toInt()>0? HIGH : LOW);
      Serial.print(key); Serial.println(cmd.toInt()>0? " 1" : " 0");
    }
    else if (cmd.startsWith("keep-alive")) { // turn on keep-alive, argument is timer interval in milliseconds (initialize() argument is long)
      cmd.remove(0, cmd.indexOf(" ")); cmd.trim();
      KA_dly = cmd.toInt() *  (long) 1000;
      KeepAlived = KA_dly > 0;
      Serial.print(":");
      Serial.readBytes(&KA_stream_len, 1);
      Serial.readBytes(KA_Stream, KA_stream_len);
      KA_Query = ((KA_Stream[3] == Get_PA_status) || (KA_Stream[3] == Get_all_data) || (KA_Stream[3] == Get_faults));
      KA_time = millis();
    }
    else if (cmd.startsWith("ka_query\n")) { // relay communications between USB and RS485
      Serial.print(":");
      if (KA_Query) {Serial.write(KA_buf_len); Serial.write(KA_Buf, KA_buf_len);}  // put on USB serial
      else Serial.write((byte) 0x00);
    }
  }
}

void QueryIterate(int n, bool ping){
    int i;
    WRITE_ENABLE;
    WRITE_ADDRESS(stream);
    for (i=1; i<stream_len; i++) {WRITE_STREAM(stream, i);}
    READ_ENABLE; 
    if (ping) Serial.print(":");
    
    // read results, closing and reopening comm slowed it down to be ~50% reliable
    READ_STREAM_BEGIN(buf[n], 0, 3); READ_STREAM_CPMLT(buf[n], 3, (buf[n][2] + 2)); buf_len = buf[n][2] + 5;
    if (buf_len>30) {digitalWrite(BNC_PIN, HIGH != POL); delay(25);}
}

void KeepAlive(void)
{
    WRITE_ENABLE; // relay communications between Arduino and RS485 (stored query, store results)
    WRITE_ADDRESS(KA_Stream);
    for (i=1; i<KA_stream_len; i++) {WRITE_STREAM(KA_Stream, i);}
    
    if (KA_Query) {
      READ_ENABLE;
      // read results
      READ_STREAM_BEGIN(KA_Buf, 0, 3); READ_STREAM_CPMLT(KA_Buf, 3, (KA_Buf[2] + 2)); KA_buf_len = KA_Buf[2] + 5;
    }
}

//---------------------------------------------------------------------------
const unsigned short CRC16_CCITT_TAB[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

//---------------------------------------------------------------------------
// Function: ProcessPacket
// Description: Calculate CCITT CRC16 for selected frame.
//---------------------------------------------------------------------------
unsigned int CalculateCRC16(unsigned char *buffer, unsigned char length)
{
    unsigned int crc;
    
    crc = 0xFFFF;

    do {
        crc=(unsigned int)((crc<<8)^CRC16_CCITT_TAB[((unsigned char)(crc>>8))^*buffer]);
        buffer++;
        length--;
    }
    while (length);
    crc ^= 0xFFFF;

    return crc;
}
