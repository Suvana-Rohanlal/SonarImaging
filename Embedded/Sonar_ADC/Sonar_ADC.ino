#include <ADC.h>
#include <ADC_util.h>
#include <DMAChannel.h>
#include <ADC_Module.h>
#include "Chirp.h"

/*
  Constants defined.
*/
#define reference_voltage 3.3
#define ADC_res 65535
#define DAC_res 4095
#define DC_offset 1
#define amp 1


#define BUFFER_SIZE 17000                  // up to 85% of dynamic memory (65,536 bytes)
#define SAMPLE_RATE 250000                   // see below maximum values
#define SAMPLE_AVERAGING 0                  // 0, 4, 8, 16 or 32
#define SAMPLING_GAIN 1                     // 1, 2, 4, 8, 16, 32 or 64
#define SAMPLE_RESOLUTION 16                // 8, 10, 12 or 16 


#define CHECKINPUT_INTERVAL   200        // 20 times per second
#define DISPLAY_INTERVAL      100000        // 10 times per second
#define SERIAL_PORT_SPEED     9600          // USB is always 12 Mbit/sec on teensy
#define DEBUG                 false

/*
  Variable definitions and initializations
*/
unsigned long lastInAvail;       //
unsigned long lastDisplay;       //
unsigned long currentTime;       //
const int readPin0             = A0;
const int readPin1             = A13;  //must be a pin belonging to ADC1
int          inByte   = 0;
volatile int first_wave =0;
int i =0;
int sample;
int count=0;
volatile uint8_t          aorb_busy  = 0;      //
volatile uint8_t          a_full     = 0;      //
volatile uint8_t          b_full     = 0;      //

ADC *adc = new ADC(); // adc object
DMAChannel dma0;
elapsedMicros time;

DMAMEM static uint16_t buf_a[BUFFER_SIZE]; // buffer a
DMAMEM static uint16_t buf_b[BUFFER_SIZE]; // buffer b
uint32_t                    freq     = SAMPLE_RATE;
uint8_t                     aver     = SAMPLE_AVERAGING;
uint8_t                      res     = SAMPLE_RESOLUTION;
uint8_t                    sgain     = SAMPLING_GAIN;
float                       Vmax     = 3.3;
ADC_REFERENCE               Vref     = ADC_REFERENCE::REF_3V3;
ADC_SAMPLING_SPEED    samp_speed     = ADC_SAMPLING_SPEED::VERY_HIGH_SPEED;
ADC_CONVERSION_SPEED  conv_speed     = ADC_CONVERSION_SPEED::VERY_HIGH_SPEED;
bool          STREAM  = false;
bool          VERBOSE = true;
bool          BINARY = true;

/////////////////////////////// SETUP ///////////////////////////////////////
void setup() {
  // put your setup code here, to run once:
  pinMode(readPin0, INPUT); 
  pinMode(readPin1, INPUT); 
  pinMode(LED_BUILTIN, OUTPUT);

  while (!Serial && millis() < 3000) ;
  Serial.begin(Serial.baud());
  Serial.println("ADC Server (Minimal)");
  Serial.println("c to start conversion, p to print buffer");

   ///// ADC0 ////
  adc->adc0->setAveraging(aver); // set number of averages
  adc->adc0->setResolution(res); // set bits of resolution
  adc->adc0->setConversionSpeed(conv_speed);//(ADC_CONVERSION_SPEED::HIGH_SPEED); // change the conversion speed
  adc->adc0->setSamplingSpeed(samp_speed);//(ADC_SAMPLING_SPEED::HIGH_SPEED); // change the sampling speed

  ////// ADC1 /////
  adc->adc1->setAveraging(aver); // set number of averages
  adc->adc1->setResolution(res); // set bits of resolution
  adc->adc1->setConversionSpeed(conv_speed);//(ADC_CONVERSION_SPEED::HIGH_SPEED); // change the conversion speed
  adc->adc1->setSamplingSpeed(samp_speed);//(ADC_SAMPLING_SPEED::HIGH_SPEED); // change the sampling speed

  
 // adc->startSynchronizedContinuous(readPin0, readPin1);
    // You can also try:
    //adc->startSynchronizedContinuousDifferential(A10, A11, A12, A13);
    // Read the values in the loop() with readSynchronizedContinuous()

  delay(100);
  analogWriteResolution(12);
  Serial.println("End Setup");
}

//////////////ADC and DMA functions///////////////////////////
void dma0_isr_single(void) {
  aorb_busy = 0;
     a_full = 1;
  dma0.clearInterrupt(); // takes more than 0.5 micro seconds
  dma0.clearComplete(); // takes about ? micro seconds
}

void ADC_setup(void){
  ///// ADC0 ////
  adc->adc0->setAveraging(aver); // set number of averages
  adc->adc0->setResolution(res); // set bits of resolution
  adc->adc0->setConversionSpeed(conv_speed);//(ADC_CONVERSION_SPEED::HIGH_SPEED); // change the conversion speed
  adc->adc0->setSamplingSpeed(samp_speed);//(ADC_SAMPLING_SPEED::HIGH_SPEED); // change the sampling speed

  ////// ADC1 /////
  adc->adc1->setAveraging(aver); // set number of averages
  adc->adc1->setResolution(res); // set bits of resolution
  adc->adc1->setConversionSpeed(conv_speed);//(ADC_CONVERSION_SPEED::HIGH_SPEED); // change the conversion speed
  adc->adc1->setSamplingSpeed(samp_speed);//(ADC_SAMPLING_SPEED::HIGH_SPEED); // change the sampling speed

  //DMA set up//
  dma0.source((volatile uint16_t&)ADC0_RA);
  dma0.destinationBuffer(buf_a, sizeof(buf_a));
  dma0.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC0);
  dma0.interruptAtCompletion();
  //dma0.disableOnCompletion();
  dma0.attachInterrupt(&dma0_isr_single);
  
}

void ADC_start(void){
    aorb_busy  = 1;
    a_full    = 0;
    b_full    = 0;

    //result = adc->readSynchronizedContinuous();
             // if using 16 bits and single-ended is necessary to typecast to unsigned,
             // otherwise values larger than 3.3/2 will be interpreted as negative
             
    //result.result_adc0 = (uint16_t)result.result_adc0;
    //result.result_adc1 = (uint16_t)result.result_adc1;

    adc->startSynchronizedSingleRead(readPin0, readPin1);
    adc->adc0->startPDB(freq);
    adc->adc0->enableDMA();
    dma0.enable();
  
}

void ADC_stop(void){
      PDB0_CH0C1 = 0; // diasble ADC0 pre triggers    
    dma0.disable();
    adc->adc0->disableDMA();
    adc->adc0->stopPDB();
    aorb_busy = 0;
}

void ADC_wait(){
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


/////////////////////////////////Print buffer functions///////////////////////////

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


///////////////////DEBUG////////////////////////////////////////////////////
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
ADC::Sync_result result;
///////////////////////////// Loop function //////////////////////////////////////
void loop() {
  // put your main code here, to run repeatedly:
  // Keep track of loop time
  currentTime = micros();
  // Commands:
  // c initiate single conversion
  // p print buffer
  float range = (amp*DAC_res)/reference_voltage;
  float DACoffset = (DC_offset*DAC_res)/reference_voltage;
  if ((currentTime-lastInAvail) >= CHECKINPUT_INTERVAL) {
    lastInAvail = currentTime;
    if (Serial.available()) {
      inByte=Serial.read();
      
      if(inByte == 's'){
          while(1)
          {
            float val = waveformsTable[first_wave][i];
            float out = val*range;
            float nOut = out + DACoffset;
            analogWrite(A21,nOut);

            i++;
            if(i==maxSamplesNum){
                i=0;
                count++;
                if(count==1){
                    count=0;
                    break;
                  }
              }

              sample = 1;
              delayMicroseconds(sample);
          }
        }
         else if (inByte == 'c') { 
             //ADC_setup();
             //ADC_start();
             //ADC_wait();
             //ADC_stop();
             //adc->printError();
            //adc->resetError();
            while()
            result = adc->readSynchronizedContinuous();

            
          }
          else if (inByte == 'p') { // print buffer
          printBuffer(buf_a, 0, BUFFER_SIZE-1);
      }
         }
  }

}
