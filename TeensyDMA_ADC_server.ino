#include <ADC.h>
#include <DMAChannel.h>

#define BUFFER_SIZE 4096                    // up to 85% of dynamic memory (65,536 bytes)
#define SAMPLE_RATE 10000                   // see below maximum values
#define SAMPLE_AVERAGING 0                  // 0, 4, 8, 16 or 32
#define SAMPLING_GAIN 1                     // 1, 2, 4, 8, 16, 32 or 64
#define SAMPLE_RESOLUTION 12                // 8, 10, 12 or 16 

// ------------------------------------
// Teensy 3.2
// ------------------------------------
// Averaging 0 ------------------------
// 16 bit 685kS/s, VERY_HIGH, VERY_HIGH
// 12 bit 708kS/s, VERY_HIGH, VERY_HIGH
//  8 bit 754kS/s, VERY_HIGH, VERY_HIGH
// 16 bit 355kS/s, HIGH16, HIGH
// 16 bit 358kS/s, HIGH, HIGH
// 12 bit 417kS/s, HIGH, HIGH
//  8 bit 470kS/s, HIGH, HIGH
// Averaging 4 ------------------------
// 16 bit 206kS/s, VERY_HIGH, VERY_HIGH
// 12 bit 250kS/s, VERY_HIGH, VERY_HIGH
// 8  bit 285kS/s, VERY_HIGH, VERY_HIGH
// ------------------------------------
// From Manual:
// short conversion time   1.45 micro seconds
// typical conversion time 3.75 micro seconds
// long conversion time    1.84 milli seconds with 32 averages of 57.62 micro seconds for each

// Main Loop Flow
#define CHECKINPUT_INTERVAL   50000         // 20 times per second
#define LEDBLINK_INTERVAL     100000        // 10 times per second
#define DISPLAY_INTERVAL      100000        // 10 times per second
#define SERIAL_PORT_SPEED     9600          // USB is always 12 Mbit/sec on teensy
#define DEBUG                 false

unsigned long lastInAvail;       //
unsigned long lastDisplay;       //
unsigned long lastBlink;         //
unsigned long currentTime;       //
bool          STREAM  = false;
bool          VERBOSE = true;
bool          BINARY = true;
// I/O-Pins
const int readPin0             = A9;
const int ledPin               = LED_BUILTIN;

//ADC & DMA Config
ADC *adc = new ADC(); //adc object
DMAChannel dma0;
// Variables for ADC0
DMAMEM static uint16_t buf_a[BUFFER_SIZE]; // buffer a
DMAMEM static uint16_t buf_b[BUFFER_SIZE]; // buffer b
volatile uint8_t          aorb_busy  = 0;      //
volatile uint8_t          a_full     = 0;      //
volatile uint8_t          b_full     = 0;      //
uint32_t                    freq     = SAMPLE_RATE;
uint8_t                     aver     = SAMPLE_AVERAGING;
uint8_t                      res     = SAMPLE_RESOLUTION;
uint8_t                    sgain     = SAMPLING_GAIN;
float                       Vmax     = 3.3;
ADC_REFERENCE               Vref     = ADC_REFERENCE::REF_3V3;
ADC_SAMPLING_SPEED    samp_speed     = ADC_SAMPLING_SPEED::VERY_HIGH_SPEED;
ADC_CONVERSION_SPEED  conv_speed     = ADC_CONVERSION_SPEED::VERY_HIGH_SPEED;

// Processing Buffer
uint16_t processed_buf[BUFFER_SIZE]; // processed data buffer

void setup() { // =====================================================

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(readPin0, INPUT); // single ended

  // Setup monitor pin
  pinMode(ledPin, OUTPUT);
  digitalWriteFast(ledPin, LOW); // LED low, setup start

  while (!Serial && millis() < 3000) ;
  Serial.begin(Serial.baud());
  Serial.println("ADC PDB & DMA Server");
  printHelp();
  
  // clear buffers
  memset((void*)buf_a, 0, sizeof(buf_a));
  memset((void*)buf_b, 0, sizeof(buf_b));
  memset((void*)processed_buf, 0, sizeof(buf_b));
  
  // LED on, setup complete
  digitalWriteFast(ledPin, HIGH);
  lastBlink = micros();

} // setup =========================================================

int          inByte   = 0;
String inNumberString = "";
long         inNumber = -1;
boolean   chunk1_sent = false;
boolean   chunk2_sent = false;
boolean   chunk3_sent = false;


void loop() { // ===================================================

  // Keep track of loop time
  currentTime = micros();

  ///////////////////////////////////////////////////////////////
  // Transmitt Data
  ///////////////////////////////////////////////////////////////
  //ADD YOUR CODE HERE
  //////////////////////////////////////////////////////////////
  if (STREAM) { 
    if (a_full == 1) { 
      // process buffer A
      ////////////////////////////////////////////////////////////////////////
      //processed_buf = something here (buf_a); <<<<<<<<<<<<<<<<<<<<<<<<<<<<<
      ////////////////////////////////////////////////////////////////////////
      // send in 4 chunks otherwise consumes all time of the loop
      if (chunk1_sent == false) {
        print2Buffer(buf_a, processed_buf, 0, floor(BUFFER_SIZE/4)-1); 
        chunk1_sent = true;
      } else if (chunk2_sent == false) {
        print2Buffer(buf_a, processed_buf, floor(BUFFER_SIZE/4), floor(BUFFER_SIZE/2)-1); 
        chunk2_sent = true;
      } else if (chunk3_sent == false) {
        print2Buffer(buf_a, processed_buf, floor(BUFFER_SIZE/2), floor(BUFFER_SIZE*3/4)-1); 
        chunk3_sent = true;
      } else {
        print2Buffer(buf_a, processed_buf, floor(BUFFER_SIZE*3/4), BUFFER_SIZE-1); 
        chunk1_sent = false;
        chunk2_sent = false;
        chunk3_sent = false;
        a_full = 0;
      }
    }
    if (b_full == 1) { 
      ////////////////////////////////////////////////////////////////////////
      //processed_buf = something here (buf_b); <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
      ////////////////////////////////////////////////////////////////////////
      if (chunk1_sent == false) {
        print2Buffer(buf_b, processed_buf, 0, floor(BUFFER_SIZE/4)-1); 
        chunk1_sent = true;
      } else if (chunk2_sent == false) {
        print2Buffer(buf_b, processed_buf, floor(BUFFER_SIZE/4), floor(BUFFER_SIZE/2)-1); 
        chunk2_sent = true;
      } else if (chunk3_sent == false) {
        print2Buffer(buf_b, processed_buf, floor(BUFFER_SIZE/2), floor(BUFFER_SIZE*3/4)-1); 
        chunk3_sent = true;
      } else {
        print2Buffer(buf_b, processed_buf, floor(BUFFER_SIZE*3/4), BUFFER_SIZE-1); 
        chunk1_sent = false;
        chunk2_sent = false;
        chunk3_sent = false;
        b_full = 0;
      }
    }
  } // Stream
  
  ///////////////////////////////////////////////////////////////
  // Input Commands
  ///////////////////////////////////////////////////////////////
  // Menu
  // f/F display/set sampling frequency
  // a/A display/set averaging  0, 4, 8, 16 or 32
  // r/R display/set resolution 8, 10, 12 or 16
  // g/G display/set gain       1, 2, 4, 8, 16, 32 or 64
  // l/L display/set Vref       1 [1.1] or 3 [3.3]
  // o/O dipslay/set conv speed 1=very low, 2=low, 3=med, 4=high16, 5=high, 6=very high, 10=2.4, 11=4.0, 12=5.2, 13=6.0
  // m/M dipslay/set samp speed 1=very low, 2=low, 3=med,           4=high, 5=very high
  // u/U display/set Vmax       12 = 1.2V
  // c/C initiate single/continous conversion
  // p print buffer
  // s/S enable/disable stream
  // v/V enable/disable verbose
  // x   stop conversion
  
  if ((currentTime-lastInAvail) >= CHECKINPUT_INTERVAL) {
    lastInAvail = currentTime;
    if (Serial.available()) {
      inByte=Serial.read();
      
      // reading numbers -------------
      if ((inByte >= '0') && (inByte <= '9')) { // integer input
        inNumberString += char(inByte); //append
      } else if (inByte =='\n') {
        Serial.println(inNumberString);
        inNumber=inNumberString.toInt();
        inNumberString = "";
        
      // reading settings commands ------------
      } else if (inByte == 'F') { // set sampling frequency 
          freq=uint32_t(inNumber);
          Serial.print("Sampling Frequency [Hz]: ");
          Serial.println(freq);
      } else if (inByte == 'f') { // display sampling frequency
          Serial.print("Sampling Frequency [Hz]: ");
          Serial.println(freq);
      } else if (inByte == 'A') { // set averaging 
          aver=uint8_t(inNumber);
          Serial.print("Averaging: ");
          Serial.println(aver);
      } else if (inByte == 'a') { // display averaging 
          Serial.print("Averaging: ");
          Serial.println(aver);
      } else if (inByte == 'R') { // set resolution 
          res=uint8_t(inNumber);
          Serial.print("Resolution [bits]: ");
          Serial.println(res);
      } else if (inByte == 'r') { // display resolution 
          Serial.print("Resolution [bits]: ");
          Serial.println(res);
      } else if (inByte == 'G') { // set gain 
          sgain = uint8_t(inNumber);
          Serial.print("Gain: ");
          Serial.println(sgain);
      } else if (inByte == 'g') { // display gain 
          Serial.print("Gain: ");
          Serial.println(sgain);
      } else if (inByte == 'U') { // set Vmax
          Vmax = float(float(inNumber)/10.0);
          Serial.print("Vmax: ");
          Serial.println(Vmax);
      } else if (inByte == 'u') { // display Vmax 
          Serial.print("Vmax: ");
          Serial.println(Vmax);
      } else if (inByte == 'L') { // set voltage reference 
          if      (inNumber==1) { Vref = ADC_REFERENCE::REF_1V2; }
          else if (inNumber==3) { Vref = ADC_REFERENCE::REF_3V3; }
          else                    Vref = ADC_REFERENCE::REF_3V3;
          Serial.print("Vref: ");
          if (     Vref == ADC_REFERENCE::REF_3V3) { Serial.println("3.3V"); }
          else if (Vref == ADC_REFERENCE::REF_1V2) { Serial.println("1.2V"); }
          else Serial.println("undefined");
      } else if (inByte == 'l') { // display volTage reference
          Serial.print("Vref: ");
          if (     Vref == ADC_REFERENCE::REF_3V3) { Serial.println("3.3V"); }
          else if (Vref == ADC_REFERENCE::REF_1V2) { Serial.println("1.2V"); }
          else Serial.println("undefined");
      } else if (inByte == 'O') { // set conversion speed 
          if      (inNumber==1)  { conv_speed = ADC_CONVERSION_SPEED::VERY_LOW_SPEED; }
          else if (inNumber==2)  { conv_speed = ADC_CONVERSION_SPEED::LOW_SPEED; }
          else if (inNumber==3)  { conv_speed = ADC_CONVERSION_SPEED::MED_SPEED; }
          else if (inNumber==4)  { conv_speed = ADC_CONVERSION_SPEED::HIGH_SPEED_16BITS; }
          else if (inNumber==5)  { conv_speed = ADC_CONVERSION_SPEED::HIGH_SPEED; }
          else if (inNumber==6)  { conv_speed = ADC_CONVERSION_SPEED::VERY_HIGH_SPEED; }
          else if (inNumber==10) { conv_speed = ADC_CONVERSION_SPEED::ADACK_2_4; }
          else if (inNumber==11) { conv_speed = ADC_CONVERSION_SPEED::ADACK_4_0; }
          else if (inNumber==12) { conv_speed = ADC_CONVERSION_SPEED::ADACK_5_2; }
          else if (inNumber==13) { conv_speed = ADC_CONVERSION_SPEED::ADACK_6_2; }
          else                     conv_speed = ADC_CONVERSION_SPEED::MED_SPEED;
          Serial.print("Conversion Speed: ");
          if (     conv_speed == ADC_CONVERSION_SPEED::VERY_LOW_SPEED)    { Serial.println("very low speed");}
          else if (conv_speed == ADC_CONVERSION_SPEED::LOW_SPEED)         { Serial.println("low speed");}
          else if (conv_speed == ADC_CONVERSION_SPEED::MED_SPEED)         { Serial.println("med speed");}
          else if (conv_speed == ADC_CONVERSION_SPEED::HIGH_SPEED_16BITS) { Serial.println("high speed 16bits");}
          else if (conv_speed == ADC_CONVERSION_SPEED::HIGH_SPEED)        { Serial.println("high speed");}
          else if (conv_speed == ADC_CONVERSION_SPEED::VERY_HIGH_SPEED)   { Serial.println("very high speed");}
          else if (conv_speed == ADC_CONVERSION_SPEED::ADACK_2_4)         { Serial.println("2.4MHz");}
          else if (conv_speed == ADC_CONVERSION_SPEED::ADACK_4_0)         { Serial.println("4.0MHz");}
          else if (conv_speed == ADC_CONVERSION_SPEED::ADACK_5_2)         { Serial.println("5.2MHz");}
          else if (conv_speed == ADC_CONVERSION_SPEED::ADACK_6_2)         { Serial.println("6.2MHz");}
          else Serial.println("undefined");
      } else if (inByte == 'o') { // display conversion speed
          Serial.print("Conversion Speed: ");
          if (     conv_speed == ADC_CONVERSION_SPEED::VERY_LOW_SPEED)    { Serial.println("very low speed");}
          else if (conv_speed == ADC_CONVERSION_SPEED::LOW_SPEED)         { Serial.println("low speed");}
          else if (conv_speed == ADC_CONVERSION_SPEED::MED_SPEED)         { Serial.println("med speed");}
          else if (conv_speed == ADC_CONVERSION_SPEED::HIGH_SPEED_16BITS) { Serial.println("high speed 16bits");}
          else if (conv_speed == ADC_CONVERSION_SPEED::HIGH_SPEED)        { Serial.println("high speed");}
          else if (conv_speed == ADC_CONVERSION_SPEED::VERY_HIGH_SPEED)   { Serial.println("very high speed");}
          else if (conv_speed == ADC_CONVERSION_SPEED::ADACK_2_4)         { Serial.println("2.4MHz");}
          else if (conv_speed == ADC_CONVERSION_SPEED::ADACK_4_0)         { Serial.println("4.0MHz");}
          else if (conv_speed == ADC_CONVERSION_SPEED::ADACK_5_2)         { Serial.println("5.2MHz");}
          else if (conv_speed == ADC_CONVERSION_SPEED::ADACK_6_2)         { Serial.println("6.2MHz");}
          else Serial.println("undefined");
      } else if (inByte == 'M') { // set sampling speed
          if      (inNumber==1)  { samp_speed = ADC_SAMPLING_SPEED::VERY_LOW_SPEED; }
          else if (inNumber==2)  { samp_speed = ADC_SAMPLING_SPEED::LOW_SPEED; }
          else if (inNumber==3)  { samp_speed = ADC_SAMPLING_SPEED::MED_SPEED; }
          else if (inNumber==4)  { samp_speed = ADC_SAMPLING_SPEED::HIGH_SPEED; }
          else if (inNumber==5)  { samp_speed = ADC_SAMPLING_SPEED::VERY_HIGH_SPEED; }
          else                     samp_speed = ADC_SAMPLING_SPEED::MED_SPEED;
          Serial.print("Sampling speed: ");
          if (     samp_speed == ADC_SAMPLING_SPEED::VERY_LOW_SPEED)  {Serial.println("very low speed");}
          else if (samp_speed == ADC_SAMPLING_SPEED::LOW_SPEED)       {Serial.println("low speed");}
          else if (samp_speed == ADC_SAMPLING_SPEED::MED_SPEED)       {Serial.println("medium speed");}
          else if (samp_speed == ADC_SAMPLING_SPEED::HIGH_SPEED)      {Serial.println("high speed");}
          else if (samp_speed == ADC_SAMPLING_SPEED::VERY_HIGH_SPEED) {Serial.println("very high speed");}
          else Serial.println("undefined");         
      } else if (inByte == 'm') { // display sampling speed
          Serial.print("Sampling speed: ");
          if (     samp_speed == ADC_SAMPLING_SPEED::VERY_LOW_SPEED)  {Serial.println("very low speed");}
          else if (samp_speed == ADC_SAMPLING_SPEED::LOW_SPEED)       {Serial.println("low speed");}
          else if (samp_speed == ADC_SAMPLING_SPEED::MED_SPEED)       {Serial.println("medium speed");}
          else if (samp_speed == ADC_SAMPLING_SPEED::HIGH_SPEED)      {Serial.println("high speed");}
          else if (samp_speed == ADC_SAMPLING_SPEED::VERY_HIGH_SPEED) {Serial.println("very high speed");}
          else Serial.println("undefined");         

      // reading execution commands ------------
      } else if (inByte == 'c') { // single block conversion
          if ((aorb_busy == 1) || (aorb_busy == 2)) { stop_ADC(); }
          setup_ADC_single();
          start_ADC();
          wait_ADC_single();
          stop_ADC();
          adc->printError();
          adc->resetError();
      } else if (inByte == 'C') { // contious conversion
          if ((aorb_busy == 1) || (aorb_busy == 2)) { stop_ADC(); }
          setup_ADC_continuous();
          start_ADC();
      } else if (inByte == 'p') { // print buffer
          printBuffer(buf_a, 0, BUFFER_SIZE-1);
      } else if (inByte == 's') { // 
          STREAM = false;  
      } else if (inByte == 'S') { // 
          STREAM = true;  
      } else if (inByte == 'v') { // 
          VERBOSE = false;  
      } else if (inByte == 'V') { // 
          VERBOSE = true;  
      } else if (inByte == 'b') { // 
          BINARY = false;  
      } else if (inByte == 'B') { // 
          BINARY = true;  
      } else if (inByte == 'x') { // 
          if (aorb_busy >0) {stop_ADC();}  
      } else if ((inByte == '?') || (inByte == 'h')) { // send HELP information
        printHelp();
      }
    } // end if serial input available
  } // end check serial in time interval
    
  if ((currentTime-lastDisplay) >= DISPLAY_INTERVAL) {
    lastDisplay = currentTime;
    adc->printError();
    adc->resetError();
  } 
    
  ///////////////////////////////////////////////////////////////
  // Blink LED 
  ///////////////////////////////////////////////////////////////
  
  // Keep LED blinking when system is ready
  if ((currentTime - lastBlink) > LEDBLINK_INTERVAL) {
    lastBlink = currentTime;
    digitalWriteFast(ledPin, !digitalReadFast(ledPin));
  } // end blink interval time

} // end loop ======================================================

// Support =========================================================
void printHelp(){
    Serial.println("--HELP---");
    Serial.println("f/F display/set sampling frequency");
    Serial.println("a/A display/set averaging  0, 4, 8, 16 or 32");
    Serial.println("r/R display/set resolution 8, 10, 12 or 16");
    Serial.println("g/G display/set gain       1, 2, 4, 8, 16, 32 or 64");
    Serial.println("l/L display/set Vref       1 [1.1] or 3 [3.3]");
    Serial.println("o/O dipslay/set conv speed 1=very low, 2=low, 3=med, 4=high16 5=high, 6=very high, 10=2.4, 11=4.0, 12=5.2, 13=6.0 MHz");
    Serial.println("m/M dipslay/set samp speed 1=very low, 2=low, 3=med,          4=high, 5=very high");
    Serial.println("u/U display/set Vmax, 12=1.2V");
    Serial.println("");
    Serial.println("c/C initiate single/continous conversion");
    Serial.println("p print buffer");
    Serial.println("s/S disable/enable stream");
    Serial.println("v/V disable/enable verbose");
    Serial.println("b/B disable/enable binary transmission");
    Serial.println("x stop ADC");
    Serial.println("--HELP END---");
}
// ADC
void setup_ADC_single(void) {
  // clear buffers
  memset((void*)buf_a, 0, sizeof(buf_a));
  // Initialize the ADC
  if (sgain >1) { adc->enablePGA(sgain, ADC_0); }  else { adc->disablePGA(ADC_0); }         
  adc->setReference(Vref, ADC_0);
  adc->setAveraging(aver); 
  adc->setResolution(res); 
  if (((Vref == ADC_REFERENCE::REF_3V3) && (Vmax > 3.29)) || ((Vref == ADC_REFERENCE::REF_1V2) && (Vmax > 1.19))) { 
    adc->disableCompare(ADC_0);
  } else if (Vref == ADC_REFERENCE::REF_3V3) {
    adc->enableCompare(Vmax/3.3*adc->getMaxValue(ADC_0), 0, ADC_0);
  } else if (Vref == ADC_REFERENCE::REF_1V2) {
    adc->enableCompare(Vmax/1.2*adc->getMaxValue(ADC_0), 0, ADC_0);    
  }
  //adc->enableCompareRange(1.0*adc->getMaxValue(ADC_1)/3.3, 2.0*adc->getMaxValue(ADC_1)/3.3, 1, 1, ADC_1); // ready if value lies out of [1.0,2.0] V
  adc->setConversionSpeed(conv_speed, ADC_0);
  adc->setSamplingSpeed(samp_speed, ADC_0);      

  // Initialize dma
  dma0.source((volatile uint16_t&)ADC0_RA);
  dma0.destinationBuffer(buf_a, sizeof(buf_a));
  dma0.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC0);
  dma0.interruptAtCompletion();
  //dma0.disableOnCompletion();
  dma0.attachInterrupt(&dma0_isr_single);
}

void setup_ADC_continuous(void) {
  // clear buffers
  memset((void*)buf_a, 0, sizeof(buf_a));
  memset((void*)buf_b, 0, sizeof(buf_b));
  // Initialize the ADC
  if (sgain > 1) { adc->enablePGA(sgain, ADC_0); } else { adc->disablePGA(ADC_0); } 
  adc->setReference(Vref, ADC_0);
  adc->setAveraging(aver); 
  adc->setResolution(res); 
  if (((Vref == ADC_REFERENCE::REF_3V3) && (Vmax > 3.29)) || ((Vref == ADC_REFERENCE::REF_1V2) && (Vmax > 1.19))) { 
    adc->disableCompare(ADC_0);
  } else if (Vref == ADC_REFERENCE::REF_3V3) {
    adc->enableCompare(Vmax/3.3*adc->getMaxValue(ADC_0), 0, ADC_0);
  } else if (Vref == ADC_REFERENCE::REF_1V2) {
    adc->enableCompare(Vmax/1.2*adc->getMaxValue(ADC_0), 0, ADC_0);    
  }
  //adc->enableCompareRange(1.0*adc->getMaxValue(ADC_1)/3.3, 2.0*adc->getMaxValue(ADC_1)/3.3, 1, 1, ADC_1); // ready if value lies out of [1.0,2.0] V
  adc->setConversionSpeed(conv_speed, ADC_0);     
  adc->setSamplingSpeed(samp_speed, ADC_0);      

  // Initialize dma
  dma0.source((volatile uint16_t&)ADC0_RA);
  dma0.destinationBuffer(buf_a, sizeof(buf_a));
  dma0.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC0);
  dma0.interruptAtCompletion();
  dma0.attachInterrupt(&dma0_isr_continuous);
}

void start_ADC(void) {
    // Start adc
    aorb_busy  = 1;
    a_full    = 0;
    b_full    = 0;
    adc->adc0->startSingleRead(readPin0);
    // frequency, hardware trigger and dma
    adc->adc0->startPDB(freq); // set ADC_SC2_ADTRG
    adc->enableDMA(ADC_0); // set ADC_SC2_DMAEN
    dma0.enable();
}

void stop_ADC(void) {
    PDB0_CH0C1 = 0; // diasble ADC0 pre triggers    
    dma0.disable();
    adc->disableDMA(ADC_0);
    adc->adc0->stopPDB();
    aorb_busy = 0;
}

void wait_ADC() {
  uint32_t   end_time = micros();
  uint32_t start_time = micros();
  while (!a_full || !b_full) {
    end_time = micros();
    if ((end_time - start_time) > 1100000) {
      Serial.printf("Timeout %d %d %d\n", a_full, b_full, aorb_busy);
      break;
    }
  }
}

void wait_ADC_single() {
  uint32_t   end_time = micros();
  uint32_t start_time = micros();
  while (!a_full) {
    end_time = micros();
    if ((end_time - start_time) > 1100000) {
      Serial.printf("Timeout %d %d\n", a_full, aorb_busy);
      break;
    }
  }
}

void dma0_isr_single(void) {
  aorb_busy = 0;
     a_full = 1;
  dma0.clearInterrupt(); // takes more than 0.5 micro seconds
  dma0.clearComplete(); // takes about ? micro seconds
}

void dma0_isr_continuous(void) {
  // this service routine should not take longer than 1.4 micro seconds
  if (aorb_busy == 1) { 
    //switch to buffer_b
       a_full = 1;
    aorb_busy = 2;
    dma0.destinationBuffer(buf_b, sizeof(buf_b)); // takes about 0.08 micro second
  } else {
    // switch to buffer_a
       b_full = 1;
    aorb_busy = 1;
    dma0.destinationBuffer(buf_a, sizeof(buf_a));
  }
  dma0.clearInterrupt(); // takes about ? micro seconds
  //dma0.clearComplete(); // takes about ? micro seconds
}

void printBuffer(uint16_t *buffer, size_t start, size_t end) {
  size_t i;
  if (VERBOSE) {
    for (i = start; i <= end; i++) { Serial.println(buffer[i]); }
  } else {
    for (i = start; i <= end; i++) {
      serial16Print(buffer[i]);
      Serial.println(); }
  }
}

void print2Buffer(uint16_t *buffer1,uint16_t *buffer2, size_t start, size_t end) {
  size_t i;
  if (VERBOSE) {
    for (i = start; i <= end; i++) { 
      Serial.print(buffer1[i]); 
      Serial.print(","); 
      Serial.println(buffer2[i]);}
  } else if (BINARY) {
    for (i = start; i <= end; i++) {
      byte* byteData1 = (byte*) buffer1[i];
      byte* byteData2 = (byte*) buffer2[i];
      byte buf[5] = {byteData1[0],byteData1[1],byteData2[0],byteData2[1],'\n'};
      Serial.write(buf,5);
    }
  } else {
    for (i = start; i <= end; i++) {
      serial16Print((buffer1[i]));
      Serial.print(",");
      serial16Print((buffer2[i]));
      Serial.println(",");
    }
  }
}

// CONVERT FLOAT TO HEX AND SEND OVER SERIAL PORT
void serialFloatPrint(float f) {
  byte * b = (byte *) &f;
  for(int i=3; i>=0; i--) {
    
    byte b1 = (b[i] >> 4) & 0x0f;
    byte b2 = (b[i] & 0x0f);
    
    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;
    
    Serial.print(c1);
    Serial.print(c2);
  }
}

// CONVERT Byte TO HEX AND SEND OVER SERIAL PORT
void serialBytePrint(byte b) {
  byte b1 = (b >> 4) & 0x0f;
  byte b2 = (b & 0x0f);

  char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
  char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;

  Serial.print(c1);
  Serial.print(c2);
}

// CONVERT 16BITS TO HEX AND SEND OVER SERIAL PORT
void serial16Print(uint16_t u) {
  byte * b = (byte *) &u;
  for(int i=1; i>=0; i--) {
    
    byte b1 = (b[i] >> 4) & 0x0f;
    byte b2 = (b[i] & 0x0f);
    
    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;
    
    Serial.print(c1);
    Serial.print(c2);
  }
}

// CONVERT Long TO HEX AND SEND OVER SERIAL PORT
void serialLongPrint(unsigned long l) {
  byte * b = (byte *) &l;
  for(int i=3; i>=0; i--) {
    
    byte b1 = (b[i] >> 4) & 0x0f;
    byte b2 = (b[i] & 0x0f);
    
    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;
    
    Serial.print(c1);
    Serial.print(c2);
  }
}
// Debug ===========================================================

typedef struct  __attribute__((packed, aligned(4))) {
  uint32_t SADDR;
  int16_t SOFF;
  uint16_t ATTR;
  uint32_t NBYTES;
  int32_t SLAST;
  uint32_t DADDR;
  int16_t DOFF;
  uint16_t CITER;
  int32_t DLASTSGA;
  uint16_t CSR;
  uint16_t BITER;
} TCD_DEBUG;

void dumpDMA_TCD(const char *psz, DMABaseClass *dmabc)
{
  Serial.printf("%s %08x %08x:", psz, (uint32_t)dmabc, (uint32_t)dmabc->TCD);
  TCD_DEBUG *tcd = (TCD_DEBUG*)dmabc->TCD;
  Serial.printf("%08x %04x %04x %08x %08x ", tcd->SADDR, tcd->SOFF, tcd->ATTR, tcd->NBYTES, tcd->SLAST);
  Serial.printf("%08x %04x %04x %08x %04x %04x\n", tcd->DADDR, tcd->DOFF, tcd->CITER, tcd->DLASTSGA,
                tcd->CSR, tcd->BITER);

}
