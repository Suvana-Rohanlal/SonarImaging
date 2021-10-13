/********************************2D Sonar - ADC Sampling********************************/
/***************************************************************************************/
/*Adapted from Jonathan Whitaker: https://github.com/johnowhitaker/teensy_adc_tips.git */
/*****************and Pedro Villanueva: https://github.com/pedvide/ADC *****************/
/***************************************************************************************/
/******************************* Done by Suvana Rohanlal *******************************/
/***************************************************************************************/

/*
  This file makes use of the ADC library originally created by Pedro Villanueva.
  Functions such as start startSynchronizedContinuos(), readSynchronizedContinuos() and 
  stopSynchronizedContinuos() was used to read the values using both ADC ports. 

  
*/

/*
  Header files
*/
#include <ADC.h>
#include <AnalogBufferDMA.h>
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
int          inByte   = 0;
volatile int first_wave =0;
int i =0;
int sample;
int count=0;
bool          STREAM  = false;
bool          VERBOSE = true;
bool          BINARY = true;

ADC *adc = new ADC(); 

DMAMEM static volatile uint16_t __attribute__((aligned(32))) dma_adc_buff1[BUFFER_SIZE];
DMAMEM static volatile uint16_t __attribute__((aligned(32))) dma_adc_buff2[BUFFER_SIZE];
AnalogBufferDMA abdma1(dma_adc_buff1, BUFFER_SIZE, dma_adc_buff2, BUFFER_SIZE);

/*
  setup function to initialize all ports/pins that are used.
*/
void setup() {
  // put your setup code here, to run once:
  pinMode(readPin0, INPUT); 
  pinMode(LED_BUILTIN, OUTPUT);

  while (!Serial && millis() < 3000) ;
  Serial.begin(Serial.baud());
  Serial.println("ADC Server (Minimal)");
  Serial.println("c to start conversion, p to print buffer");

  adc->adc0->setAveraging(8); // set number of averages
  adc->adc0->setResolution(12); // set bits of resolution
  
  abdma1.init(adc, ADC_0);
  adc->adc0->startContinuous(readPin0);

  Serial.println("End Setup");
}


/*
  Printing over serial port
*/
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




/*
  ADC processing
*/
void ProcessAnalogData(AnalogBufferDMA *pabdma, int8_t adc_num) {
  uint32_t sum_values = 0;
  uint16_t min_val = 0xffff;
  uint16_t max_val = 0;

  uint32_t average_value = pabdma->userData();

  volatile uint16_t *pbuffer = pabdma->bufferLastISRFilled();
  volatile uint16_t *end_pbuffer = pbuffer + pabdma->bufferCountLastISRFilled();

  float sum_delta_sq = 0.0;
  if ((uint32_t)pbuffer >= 0x20200000u)  arm_dcache_delete((void*)pbuffer, sizeof(dma_adc_buff1));
  while (pbuffer < end_pbuffer) {
    if (*pbuffer < min_val) min_val = *pbuffer;
    if (*pbuffer > max_val) max_val = *pbuffer;
    sum_values += *pbuffer;
    int delta_from_center = (int) * pbuffer - average_value;
    sum_delta_sq += delta_from_center * delta_from_center;

    pbuffer++;
  }

  int rms = sqrt(sum_delta_sq / BUFFER_SIZE);
  average_value = sum_values / BUFFER_SIZE;
  Serial.printf(" %d - %u(%u): %u <= %u <= %u %d ", adc_num, pabdma->interruptCount(), pabdma->interruptDeltaTime(), min_val,
                average_value, max_val, rms);
  pabdma->clearInterrupt();

  pabdma->userData(average_value);
}

/*
  Loop function to continuously run. Contains the user input to transmit 
  the chirp and the process of reading using the ADCs. 
*/
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
         else if (inByte == 'c') { // single block conversion
           if ( abdma1.interrupted()) {
            ProcessAnalogData(&abdma1, 0);
            Serial.println();
            }
         }
         
  }
}
}
