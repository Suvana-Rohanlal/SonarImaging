#include <ADC.h>
#include <DMAChannel.h>
#include <ADC_util.h>
#include "Chirp.h"

//#define oneHzSample 1000000/maxSamplesNum  // sample for the 1Hz signal expressed in microseconds 
#define refVoltage 3.3
#define DACresolution 4095 //2^n - 1
#define ADCresolution 65535//2^16-1
#define desiredOffset 1 //1.5
#define desiredAmplitude 1 //1.5

#define BUFFER_SIZE 17000                  // up to 85% of dynamic memory (65,536 bytes) 17000
#define SAMPLE_RATE 250000                   // see below maximum values //250000 400000 desirable
#define SAMPLE_AVERAGING 0                  // 0, 4, 8, 16 or 32
#define SAMPLING_GAIN 1                     // 1, 2, 4, 8, 16, 32 or 64
#define SAMPLE_RESOLUTION 16                // 8, 10, 12 or 16 



// Main Loop Flow
#define CHECKINPUT_INTERVAL   200         // 20 times per second .  50 000 8000 5000 3000
#define DISPLAY_INTERVAL      100000        // 10 times per second
#define SERIAL_PORT_SPEED     9600          // USB is always 12 Mbit/sec on teensy
#define DEBUG                 false


unsigned long lastInAvail;       //
unsigned long lastDisplay;       //
unsigned long lastBlink;         //
unsigned long currentTime;       //
unsigned long func_timer; // <<<<<<<<<<< Time execution of different functions
bool          STREAM  = false;
bool          VERBOSE = true;
bool          BINARY = true;
// I/O-Pins
const int readPin0             = A0;
const int readPin1             = A13;  //must be a pin belonging to ADC1
const int ledPin               = LED_BUILTIN;

//Added variables
float convertval;
volatile int wave0 = 0, wave1 = 0;
int i = 0;
int sample;
int val;
char command;
int counter = 0;

//ADC & DMA Config
ADC *adc = new ADC(); //adc object
DMAChannel dma0;
DMAChannel dma1;
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


// Variables for ADC1
DMAMEM static uint16_t buf_a1[BUFFER_SIZE]; // buffer a //uint16_t
DMAMEM static uint16_t buf_b1[BUFFER_SIZE]; // buffer b
volatile uint8_t          aorb1_busy  = 0;      //
volatile uint8_t          a1_full     = 0;      //
volatile uint8_t          b1_full     = 0;      //


// Processing Buffer
uint16_t processed_buf[BUFFER_SIZE]; // processed data buffer
uint16_t processed_buf1[BUFFER_SIZE]; // processed data buffer

void setup() { // =====================================================

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(readPin0, INPUT); // Channel 0
  pinMode(readPin1, INPUT); // Channel 1

  // Setup monitor pin
  pinMode(ledPin, OUTPUT);
  digitalWriteFast(ledPin, LOW); // LED low, setup start

  while (!Serial && millis() < 3000) ;
  Serial.begin(Serial.baud());
  Serial.println("ADC Server (Minimal)");
  Serial.println("c to start conversion, p to print buffer");
  
  // clear buffers
  memset((void*)buf_a, 0, sizeof(buf_a));
  memset((void*)buf_b, 0, sizeof(buf_b));
  memset((void*)processed_buf, 0, sizeof(buf_b));

   // clear buffers for channel 1
  memset((void*)buf_a1, 0, sizeof(buf_a1));
  memset((void*)buf_b1, 0, sizeof(buf_b1));
  memset((void*)processed_buf1, 0, sizeof(buf_b1));
  

  //modified here
  analogWriteResolution(12);  // set the analog output resolution to 12 bit (4096 levels)

} // setup =========================================================

int          inByte   = 0;
String inNumberString = "";
long         inNumber = -1;
boolean   chunk1_sent = false;
boolean   chunk2_sent = false;
boolean   chunk3_sent = false;



// CONVERT 16BITS TO HEX AND SEND OVER SERIAL PORT
void serial16Print(uint16_t u) {
  byte * b = (byte *) &u;
  Serial.println("Serial16Print");
  for(int i=1; i>=0; i--) {
    
    byte b1 = (b[i] >> 4) & 0x0f;
    byte b2 = (b[i] & 0x0f);
    
    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;
    
    Serial.print(c1);
    Serial.print(c2);
  }
}


void dma0_isr_single(void) {
  aorb_busy = 0;
     a_full = 1;
  dma0.clearInterrupt(); // takes more than 0.5 micro seconds
  dma0.clearComplete(); // takes about ? micro seconds
}

void dma1_isr_single(void) {
  aorb1_busy = 0;
  a1_full = 1;
  dma1.clearInterrupt(); // takes more than 0.5 micro seconds
  dma1.clearComplete(); // takes about ? micro seconds
}


// ADC
void setup_ADC_single(void) {
  // clear buffers
  memset((void*)buf_a, 0, sizeof(buf_a));

  // clear buffers for channel 1
  memset((void*)buf_a1, 0, sizeof(buf_a1));

  
  // Initialize the ADC
  //if (sgain >1) { adc->adc0->enablePGA(sgain); }  else { adc->adc0->disablePGA(); }         
  adc->adc0->setReference(Vref);
  adc->adc0->setAveraging(aver); 
  adc->adc0->setResolution(res); 
  if (((Vref == ADC_REFERENCE::REF_3V3) && (Vmax > 3.29)) || ((Vref == ADC_REFERENCE::REF_1V2) && (Vmax > 1.19))) { 
    adc->adc0->disableCompare();
  } else if (Vref == ADC_REFERENCE::REF_3V3) {
    adc->adc0->enableCompare(Vmax/3.3*adc->adc0->getMaxValue(), 0);
  } else if (Vref == ADC_REFERENCE::REF_1V2) {
    adc->adc0->enableCompare(Vmax/1.2*adc->adc0->getMaxValue(), 0);    
  }
  //adc->enableCompareRange(1.0*adc->getMaxValue(ADC_1)/3.3, 2.0*adc->getMaxValue(ADC_1)/3.3, 1, 1, ADC_1); // ready if value lies out of [1.0,2.0] V
  adc->adc0->setConversionSpeed(conv_speed);
  adc->adc0->setSamplingSpeed(samp_speed);  


    // Initialize the ADC for channel 1
 // if (sgain >1) { adc->enablePGA(sgain, ADC_1); }  else { adc->disablePGA(ADC_1); }         
  adc->adc1->setReference(Vref);
  adc->adc1->setAveraging(aver); 
  //adc->setResolution(res); 
  adc->adc1->setResolution(res); 
  if (((Vref == ADC_REFERENCE::REF_3V3) && (Vmax > 3.29)) || ((Vref == ADC_REFERENCE::REF_1V2) && (Vmax > 1.19))) { 
    adc->adc1->disableCompare();
  } else if (Vref == ADC_REFERENCE::REF_3V3) {
    adc->adc1->enableCompare(Vmax/3.3*adc->adc1->getMaxValue(), 0);
  } else if (Vref == ADC_REFERENCE::REF_1V2) {
    adc->adc1->enableCompare(Vmax/1.2*adc->adc1->getMaxValue(), 0);    
  }
  //adc->enableCompareRange(1.0*adc->getMaxValue(ADC_1)/3.3, 2.0*adc->getMaxValue(ADC_1)/3.3, 1, 1, ADC_1); // ready if value lies out of [1.0,2.0] V
  adc->adc1->setConversionSpeed(conv_speed);
  adc->adc1->setSamplingSpeed(samp_speed);   
  

  // Initialize dma
  dma0.source((volatile uint16_t&)ADC0_RA);
  dma0.destinationBuffer(buf_a, sizeof(buf_a));
  dma0.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC0);
  dma0.interruptAtCompletion();
  //dma0.disableOnCompletion();
  dma0.attachInterrupt(&dma0_isr_single);


   // Initialize dma
  dma1.source((volatile uint16_t&)ADC1_RA);
  dma1.destinationBuffer(buf_a1, sizeof(buf_a1));
  dma1.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC1);
  dma1.interruptAtCompletion();
  //dma0.disableOnCompletion();
  dma1.attachInterrupt(&dma1_isr_single);
}

void start_ADC(void) {
    // Start adc
    aorb_busy  = 1;
    a_full    = 0;
    b_full    = 0;

    //Channel 1
    aorb1_busy  = 1;
    a1_full    = 0;
    b1_full    = 0;

    //Channel 0
    adc->adc0->startSingleRead(readPin0);  //might need to change to startSynchronizedSingleRead . (used to be startSingleRead)
    // frequency, hardware trigger and dma
    adc->adc0->startPDB(freq); // set ADC_SC2_ADTRG
    adc->adc0->enableDMA(); // set ADC_SC2_DMAEN
    dma0.enable();

    //Serial.print();
    //Channel 1
    adc->adc1->startSingleRead(readPin1);
    // frequency, hardware trigger and dma
    adc->adc1->startPDB(freq); // set ADC_SC2_ADTRG
    adc->adc1->enableDMA(); // set ADC_SC2_DMAEN
    dma1.enable();
}

void stop_ADC(void) {
    PDB0_CH0C1 = 0; // diasble ADC0 pre triggers    
    dma0.disable();
    adc->adc0->disableDMA();
    adc->adc0->stopPDB();
    aorb_busy = 0;


    //Channel 1
    PDB0_CH1C1 = 0; // may need to change 
    dma1.disable();
    adc->adc1->disableDMA();
    adc->adc1->stopPDB();
    aorb1_busy = 0;
}

void wait_ADC_single() {
  uint32_t   end_time = micros();
  uint32_t start_time = micros();
  while ((!a_full)&(!a1_full)) {
    end_time = micros();
    if ((end_time - start_time) > 1100000) { //original value = 1100000
      Serial.printf("Timeout %d %d\n", a_full, aorb_busy);
      break;
    }
  }
  //Serial.printf("Conversion complete in %d us\n", end_time-start_time);
}




//modified
void printBuffer(uint16_t *buffer, size_t start, size_t end) { //change to float array??
  size_t i;
  if (VERBOSE) {
   // Serial.println("PrintBuffer if");
    for (i = start; i <= end; i++) { 
      //convertval = (buffer[i]*refVoltage)/ADCresolution;
      //uint16_t n = buffer[i];
      //uint8_t first = n>>8;
      //Serial.write(uint8_t(first));
      //uint8_t second = n&255;
      //Serial.write(uint8_t(second));
      //float test = (buffer[i]*3.3)/65535; //TAKE THIS OUT
         // v =  (v .* 3.3) / 65535
      Serial.println(buffer[i]);} //prints correctyl in serial monitor
      //Serial.println(test);} //prints correctyl in serial monitor . CHANGE THIS BACK!!
    
  } else {
    Serial.println("PrintBuffer else");
    for (i = start; i <= end; i++) {
      //convertval = (buffer[i]*refVoltage)/ADCresolution;
      serial16Print(buffer[i]);
      Serial.println(); }
  }
}


//may use this method to print both arrays??
void print2Buffer(uint16_t *buffer1,uint16_t *buffer2, size_t start, size_t end) {
  size_t i;
 // Serial.println("PrintBuffer2 if");
  if (VERBOSE) {
    for (i = start; i <= end; i++) { 
      Serial.print(buffer1[i]); 
      Serial.print(","); 
      Serial.println(buffer2[i]);}
  } else if (BINARY) {
      Serial.println("PrintBuffer2 else if");
    for (i = start; i <= end; i++) {
      byte* byteData1 = (byte*) buffer1[i];
      byte* byteData2 = (byte*) buffer2[i];
      byte buf[5] = {byteData1[0],byteData1[1],byteData2[0],byteData2[1],'\n'};
      Serial.write(buf,5);
    }
  } else {
      Serial.println("PrintBuffer2 else");
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
  //  Serial.println("serialFloatprint");
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
   // Serial.println("SerialByteprint");
  byte b1 = (b >> 4) & 0x0f;
  byte b2 = (b & 0x0f);

  char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
  char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;

  Serial.print(c1);
  Serial.print(c2);
}

// CONVERT Long TO HEX AND SEND OVER SERIAL PORT
void serialLongPrint(unsigned long l) {
   // Serial.println("SerialLongprint");
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
   // Serial.println("dumpDMA");
  Serial.printf("%s %08x %08x:", psz, (uint32_t)dmabc, (uint32_t)dmabc->TCD);
  TCD_DEBUG *tcd = (TCD_DEBUG*)dmabc->TCD;
  Serial.printf("%08x %04x %04x %08x %08x ", tcd->SADDR, tcd->SOFF, tcd->ATTR, tcd->NBYTES, tcd->SLAST);
  Serial.printf("%08x %04x %04x %08x %04x %04x\n", tcd->DADDR, tcd->DOFF, tcd->CITER, tcd->DLASTSGA,
                tcd->CSR, tcd->BITER);

}


void loop() { // ===================================================



  // Read both channels in simultaneously 
  // Print arrays to buffer one after the other 
  // Keep track of loop time
  currentTime = micros();
  // Commands:
  // s send chirp
  // c initiate single conversion
  // p print buffer
  
  if ((currentTime-lastInAvail) >= CHECKINPUT_INTERVAL) {
    lastInAvail = currentTime;
    
    if (Serial.available()) {
      inByte=Serial.read();
      Serial.println(inByte);

       if(inByte == 's')
        {
          //Serial.println("Sending Chirp");
          //for(i=1; i<maxSamplesNum; i++) 
          while(1)
          {
            float val = waveformsTable[wave0][i];
           // float rangeMultiplier = (desiredAmplitude*DACresolution)/refVoltage;
           // float out = val*rangeMultiplier; //1860 - range.amplitude of 1.5V, 1240 - range up to 1V, 2480 - range up to 2V 
           // float DACoffset = (desiredOffset*DACresolution)/refVoltage;
           // float newOut = out + DACoffset; //add offset (1860 = 1.5V) (1240 = 1V)
            analogWrite(A21, val);//newOut);  // write the selected waveform on DAC0
             // Serial.println(val);
            i++;
            if(i == 2500)//maxSamplesNum) //2000)//used to be maxNumSamples
            {
              i = 0;
              counter++;
              if(counter==1) //change back to 5
              {
                //Serial.println("Sent");
                counter=0;
                break;
              }
            }
         
            sample = 1;//0.0001 //0.000025 //9
            delayMicroseconds(sample);  // Hold the sample value for the sample time
           }
          }
      
      else if (inByte == 'c') { // single block conversion - for two channels 
          if ((aorb_busy == 1) || (aorb_busy == 2)) { stop_ADC(); }
          //setup_ADC_single();
          //start_ADC();
          setup_ADC_single();
          //Serial.println("set up complete");
          start_ADC();
          //Serial.println("start complete");
          wait_ADC_single();
          //Serial.println("wait complete");
          stop_ADC();
          //Serial.println("stop complete");
//          adc->printError();
          if(adc->adc0->fail_flag != ADC_ERROR::CLEAR) {
                Serial.print("ADC0: "); Serial.println(getStringADCError(adc->adc0->fail_flag));
            }
          adc->resetError();
      } else if (inByte == 'p') { // print buffer
        //  Serial.println(sizeof(buf_a));
          printBuffer(buf_a, 0, BUFFER_SIZE-1);
      }
      else if (inByte == 'a') { // print second buffer
          printBuffer(buf_a1, 0, BUFFER_SIZE-1);
      }
      
    } // end if serial input available
  } // end check serial in time interval
    
  if ((currentTime-lastDisplay) >= DISPLAY_INTERVAL) {
    lastDisplay = currentTime;
//    adc->printError();
    adc->resetError();
  } 
    
  

} // end loop ======================================================
