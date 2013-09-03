/* Arduino Uno Based Kyub
 *
 *
 *
 * Background Info:
 * ADC 8-Bit Mode 
 * Analog input 0 (ADC0) is used to sample the accelerometer signal
 * 
 
 */

//*******Defines/Includes
//!!deprecated, consider replacing
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#include <EEPROM.h>		//the eeprom library

//*******Function prototypes
void noteOn ();		//play MIDI note

//*******Global Variables

int consolemidimode=1; //midimode=1 consolemode=0 accelerometer=3

//pin assignments
//int LEDpin=13;
int driverpin = 13; //common pad drive pin
int zpin=0; //acelerometer axes
int xpin=1;
int ypin=2;


//cap sensing variables
int firsttime=0;  //trigger for calibration
int pad[11];  //array holding sensing pad numbers
int remap[11];  //array to simplify remapping
int cap_calibration[11];  //calibration value for each pad
long int chargetime[11];  //sensed charge time for each pad
int padstate[11];  //state of pad as touched (1) or not (0)
int padmode[11];  //state of pad touch as it is processed
//   0 = ready for new pad touch
//   -1 = have touch, waiting for volume
//   2 = have volume, waiting to be played (note on)
//   -2 = played, waiting to be turned off (note off)

long int padlasttime[11];  //last time pad was triggered
int padlastchannel[11];  //last channel held to turn right note off after key changes
int padlastnote[11];  //last note held to turn right note off after key change
int padvolume[11];  //current note volume
int pnum=0; //index for pads through each loop
int overflow=0;
unsigned long starttime=0;
unsigned long grabtime;
int hysteresishigh=25; //turn on threshold for touch
int hysteresislow=20; //turn off threshold for touch

//midi variables--arbitarary starting point
//int chord0=0;
int chord1=0;  //determined by pads 9 and 10
int chord2=0;
int channel=0x93; 
int note0=64;
int note1=64;
int note2=66;
int note3=69;
int note4=69;
int note5=71;
int note6=67;
int note7=75;
int note8=61;
int note9=62;
int notevolume=0;
int volume=0;
int pitch=0;
long int min_note_duration=100000;
long int holdoff[11]; //??

//debug printout variable
long int next=0;

//accelerometer variables
int once=0; //controls sample acquisitions
int circularaccbufferx[100]; //holds samples of A/D taken before and after pad hit
int circularaccbuffery[100];
int circularaccbufferz[100];
int circbuffpointer=0;
int triggerpoint=0; //time of pad hit
int acc_calibrationx=0; //A/D calibration values (may not be needed)
int acc_calibrationy=0;
int acc_calibrationz=0;
int xaxispeak=0; //peaks and valleys of acceleration waveforms
int xaxisvalley=0;
int yaxispeak=0;
int yaxisvalley=0;
int zaxispeak=0;
int zaxisvalley=0;

//********Functions (Subroutines)

//  plays a MIDI note.  Doesn't range check
void noteOn(int cmd, int pitch, int velocity) {
  if (consolemidimode==1)
  {
    Serial.write(cmd);
    Serial.write(pitch);
    Serial.write(velocity);
  }
}

//********Setup*************************************************

void setup ()
{

  //  Set MIDI baud rate:
  if (consolemidimode==1) Serial.begin(31250);
  // open serial comm
  if ((consolemidimode==0)|| (consolemidimode==3)) Serial.begin(9600); 

  // initialize the Arduino pins
  remap[0]=2;  //pad pin assignments--arbitary according to PCB
  remap[1]=3;
  remap[2]=4;
  remap[3]=5;
  remap[4]=6;
  remap[5]=7;
  remap[6]=8;
  remap[7]=9;
  remap[8]=10;
  remap[9]=11;
  remap[10]=12;

  pad[0]=remap[0];  //pad pin assignments--arbitary according to wiring of pads
  pad[1]=remap[1];
  pad[2]=remap[2];
  pad[3]=remap[3];
  pad[4]=remap[4];
  pad[5]=remap[5];
  pad[6]=remap[6];
  pad[7]=remap[7];
  pad[8]=remap[8];
  pad[9]=remap[9];
  pad[10]=remap[10];

  //pinMode(LEDpin, OUTPUT);
  pinMode(driverpin, OUTPUT);
  pinMode(pad[0], INPUT);
  pinMode(pad[1], INPUT);
  pinMode(pad[2], INPUT);
  pinMode(pad[3], INPUT);
  pinMode(pad[4], INPUT);
  pinMode(pad[5], INPUT);
  pinMode(pad[6], INPUT);
  pinMode(pad[7], INPUT);
  pinMode(pad[8], INPUT);
  pinMode(pad[9], INPUT);
  pinMode(pad[10], INPUT);

  //turn off pullup resistors--should not be needed
  digitalWrite(pad[0], LOW);
  digitalWrite(pad[1], LOW); 
  digitalWrite(pad[2], LOW);
  digitalWrite(pad[3], LOW);
  digitalWrite(pad[4], LOW);
  digitalWrite(pad[5], LOW);
  digitalWrite(pad[6], LOW);
  digitalWrite(pad[7], LOW);
  digitalWrite(pad[8], LOW);
  digitalWrite(pad[9], LOW);
  digitalWrite(pad[10], LOW);

  // *************Some Bit-Banging to get High Speed A/D operation*********************
  // This is basically assembly language accommodated by the Arduino IDE
  // The acronyms here are either (1) register names (e.g., ADCSRA is A/D register A)
  // (see Atmel-168 datasheet for register names and use)
  // or (2) defines/constants (e.g., ADPS2=bit 2) set in the iom168p.h header file buried in the Arduino download
  // The defines are also described in the Atmel168 datasheet (do a search of the .pdf)

  // set adc prescaler to 32 for 33kHz sampling frequency (16mhz/32/15 instructions per conversion)
  sbi (ADCSRA, ADPS2);		// This code sets the AD clock prescalar to /32 (16MHz/32=500kHz
  cbi (ADCSRA, ADPS1);		// The A/D clock should be between 50-200 kHz (+ some only 8-bit conversions!)
  sbi (ADCSRA, ADPS0);          // Normal conversion requires about 15 clock cycles so 33Khz conversion rate
  sbi (ADMUX, ADLAR);		// Left justifies A/D output so it can be read in one swoop
  sbi (ADMUX, REFS0);		// Selects VCC Reference for A/D
  cbi (ADMUX, REFS1);
}


//*********************************************************************
//*********************************************************************
//*******Main loop--read capacitive pads and accelerometer ***********
//*********************************************************************
//*********************************************************************

void loop ()
{ //main loop

    //set chord pallet according to pressing of pads 5 and 7

  int chordpallet=1;  //2=scale

  if (chordpallet==0)//*************************************************
  {
    // I, V, vi, IV switched by pads
    if ((chord1==1) && (chord2==0)) //c
    {
      channel=0x93;  
      note0=48;
      note1=60;
      note2=52;
      note3=64;
      note4=55;
      note5=72;
      note6=67;
      note7=76;
      note8=84;
    }
    if ((chord1==0) &&(chord2==1))//g
    {
      channel=0x93;  
      note0=55;
      note1=67;
      note2=59;
      note3=71;
      note4=62;
      note5=79;
      note6=74;
      note7=83;
      note8=81;
    }
    if ((chord1==1) &&(chord2==1))//am
    {
      channel=0x93;  
      note0=57;
      note1=69;
      note2=60;
      note3=72;
      note4=64;
      note5=81;
      note6=76;
      note7=84;
      note8=88;
    }

    if ((chord1==0) &&(chord2==0))//f
    {
      channel=0x93;  
      note0=53;
      note1=65;
      note2=57;
      note3=69;
      note4=60;
      note5=77;
      note6=72;
      note7=71;
      note8=84;
    }
  }

 
 
 
if (chordpallet==1)//*************************************************
  {
    // I, vi, ii, V switched by pads
    if ((chord1==1) && (chord2==0)) //c
    {
      channel=0x93;  
      note0=48;
      note1=60;
      note2=52;
      note3=64;
      note4=55;
      note5=72;
      note6=67;
      note7=76;
      note8=84;
    }
    if ((chord1==0) &&(chord2==1))//g
    {
      channel=0x93;  
      note0=55;
      note1=67;
      note2=59;
      note3=71;
      note4=62;
      note5=79;
      note6=74;
      note7=83;
      note8=81;
    }
    if ((chord1==1) &&(chord2==1))//am
    {
      channel=0x93;  
      note0=57;
      note1=69;
      note2=60;
      note3=72;
      note4=64;
      note5=81;
      note6=76;
      note7=84;
      note8=88;
    }

    if ((chord1==0) &&(chord2==0))//dm
    {
      channel=0x93;  
      note0=50;
      note1=62;
      note2=53;
      note3=65;
      note4=69;
      note5=57;
      note6=74;
      note7=77;
      note8=81;
    }
  }

if (chordpallet==2)//*************************************************
  {
    // simple major scale
     if ((chord1==1) && (chord2==0)) //c
    {
      channel=0x93;  
      note0=65;  //F
      note1=69;  //A
      note2=67;  //G
      note3=62;  //D
      note4=64;  //E
      note5=72; //C 
      note6=70;  //A#/Bb
      note7=77;  //F'
      note8=77;
    }
    if ((chord1==0) &&(chord2==1))//g
    {
      channel=0x93;  
      note0=74;
      note1=67;
      note2=62;
      note3=60;
      note4=72;
      note5=69;
      note6=64;
      note7=65;
      note8=71;
    }
    if ((chord1==1) &&(chord2==1))//am
    {
      channel=0x93;  
      note0=98;
      note1=91;
      note2=86;
      note3=84;
      note4=96;
      note5=93;
      note6=88;
      note7=89;
      note8=95;
    }

    if ((chord1==0) &&(chord2==0))//dm
    {
      channel=0x93;  
      note0=62;
      note1=55;
      note2=50;
      note3=48;
      note4=60;
      note5=57;
      note6=2;
      note7=53;
      note8=59;
    }
  }

 
 
 

if (chordpallet==3)//*************************************************
  {
    // circle of fifths
     if ((chord1==1) && (chord2==0)) //c
    {
      channel=0x93;  
      note0=86;
      note1=79;
      note2=74;
      note3=72;
      note4=84;
      note5=81;
      note6=76;
      note7=77;
      note8=83;
    }
    if ((chord1==0) &&(chord2==1))//g
    {
      channel=0x93;  
      note0=74;
      note1=67;
      note2=62;
      note3=60;
      note4=72;
      note5=69;
      note6=64;
      note7=65;
      note8=71;
    }
    if ((chord1==1) &&(chord2==1))//am
    {
      channel=0x93;  
      note0=98;
      note1=91;
      note2=86;
      note3=84;
      note4=96;
      note5=93;
      note6=88;
      note7=89;
      note8=95;
    }

    if ((chord1==0) &&(chord2==0))//dm
    {//Dr. Oct Rex controller
      channel=0x93;  
      note0=104;
      note1=76;
      note2=55;
      note3=49;
      note4=97;
      note5=83;
      note6=62;
      note7=70;
      note8=90;
    }
  } 
 
 
 if (chordpallet==4)//*************************************************
  {
   
      channel=0x93;  
      note0=28;
      note1=29;
      note2=30;
      note3=31;
      note4=32;
      note5=33;
      note6=34;
      note7=35;
      note8=36;
    }
 
 
 
  //pad calibration early after boot
  if (firsttime<20) firsttime++; 
  if (firsttime==19) for (int x=0; x<12; x++) cap_calibration[x]=chargetime[x];    

  //******************************************************************************************************
  //loop through each of 11 pads according to pnum
  //******************************************************************************************************
  if (pnum<10) pnum++; 
  else pnum=0;   
  overflow=0;

  //start touch sensing for selected pad ****************************
  //for high speed, read A/D for x, y, and z interleaved at times of necessary delay

  if (circbuffpointer<99) circbuffpointer++; 
  else circbuffpointer=0; //acell. axis circular buffer pointer

  //first measure charge up time, then measure fall time to cut sensitivity to gate threshold level
  //set driver pin high and measure rise time of selected pad
  digitalWrite(driverpin, HIGH);   // common driver pin high
  once=0;
  starttime = micros();
  while ((digitalRead(pad[pnum])==LOW) && (overflow==0))  //digital read is pretty slow it seems
  { //charge while loop
    if (micros()-starttime>3000) 
    {
      overflow=1;
      if (consolemidimode==0) //debug output to console
      {
        Serial.print ("overflow up on pin:"); 
        Serial.print(pnum);
        Serial.println(""); 
      } 
    }
  } //end charge while loop

  grabtime= micros()-starttime;
  //finish charging of input pin to full voltage
  digitalWrite(pad[pnum],HIGH);  //set pullup resistor to on
  //*********************interleaved x-axis accelerometer read **************************************
  if (once==0)
  { 
    once=1;
    cbi (ADMUX, MUX0);	// Set Input Multiplexer to Channel 0 for x-axis
    cbi (ADMUX, MUX1);	// MUX0-3 select one of 8 A/D inputs  Mux0=LSMB
    cbi (ADMUX, MUX2);
    cbi (ADMUX, MUX3); 
    sbi (ADCSRA, ADSC);	// start next conversion
    delayMicroseconds(30);  //a/d conversion time about 26us?
    circularaccbufferx[circbuffpointer]= ADCH;//analogRead(xpin);  
  } 
  //*********************interleave x-axis end****************************************************** 

  delayMicroseconds(100);//not needed if have delay from A/D
  digitalWrite(pad[pnum],LOW); //turn off pull up resistor

  //*********************interleaved y-axis accelerometer read **************************************
  once=0;
  if (once==0)
  { 
    once=1;
    sbi (ADMUX, MUX0);	// Set Input Multiplexer to Channel 1 for y-axis
    cbi (ADMUX, MUX1);	
    cbi (ADMUX, MUX2);
    cbi (ADMUX, MUX3);
    sbi (ADCSRA, ADSC);	// start next conversion
    delayMicroseconds(30);  
    circularaccbuffery[circbuffpointer]= ADCH;
  } 
  //*********************interleave y-axis end ****************************************************** 

  //set driver pin low and measure fall time of selected pad
  digitalWrite(driverpin, LOW);   
  starttime = micros();
  once=0;
  while ((digitalRead(pad[pnum])==HIGH) && (overflow==0))
  { //discharging while loop
    if (micros()-starttime>3000) 
    {
      overflow=1;
      if (consolemidimode==0) //debug mode console output
      {
        Serial.print ("overflow down on pin:"); 
        Serial.print(pnum);
        Serial.println("");
      } 
    }
  } //end discharging while loops

  grabtime= grabtime+ micros()-starttime;  //add rise and fall times together
  //*********************interleaved z-axis accelerometer read **************************************
  if (once==0)
  { 
    //had to mainline code to go fast enough
    once=1;
    cbi (ADMUX, MUX0);	// Set Input Multiplexer to Channel 2 for z-axis
    sbi (ADMUX, MUX1);	
    cbi (ADMUX, MUX2);
    cbi (ADMUX, MUX3); 
    sbi (ADCSRA, ADSC);	// start next conversion
    delayMicroseconds(30);
    circularaccbufferz[circbuffpointer]= ADCH; 
  }
  //*********************interleave #3 ******************************************************  

  delayMicroseconds(100); //to obtain the benefit of rise and fall measurements, must hit zero volta here
  chargetime[pnum]=grabtime;
  //*************************end touch sensing for given pad *****************************

  //touch detected for the first time****************************
  if (chargetime[pnum]-cap_calibration[pnum]>hysteresishigh) padstate[pnum]=1; 
  else if (chargetime[pnum]-cap_calibration[pnum]<hysteresislow) padstate[pnum]=0;

  //if (padstate[8]==1) chord0=1; //special non note playing pads that select chords
  //else chord0=0;
  if (padstate[9]==1) chord1=1;
  else chord1=0;
  if (padstate[10]==1) chord2=1; 
  else chord2=0;

  // padmode[8]=-3; //deactivate these pins for all other functions
  padmode[9]=-3;
  padmode[10]=-3;

  if ((padmode[pnum]==0) && (padstate[pnum]==1)) //ready for new note
  {
    padlasttime[pnum]=micros();  //keep this low as long as pad is held
    triggerpoint=0; 
    padmode[pnum]= -1; //-1 marks pending note before volume is determined

    //digitalWrite(LEDpin, HIGH);
    holdoff[pnum]=micros();  //?? needed ??hold off stops rapid second trigger "bounce"
  }

  if (triggerpoint<99) triggerpoint++;
  //let buffer run a bit then find max and load it into pending notes
  if (triggerpoint==50) //half of buffer
  {
    yaxispeak=circularaccbuffery[0];
    yaxisvalley=circularaccbuffery[0];
    xaxispeak=circularaccbufferx[0];
    xaxisvalley=circularaccbufferx[0];
    zaxispeak=circularaccbufferz[0];
    zaxisvalley=circularaccbufferz[0];
    acc_calibrationx=0; //calibration values will be average value of circular buffer
    acc_calibrationy=0;
    acc_calibrationz=0;


    for (int x=0; x<99; x++)  //grab peaks and valles of 100 samples of accelerometer
    {
      acc_calibrationx+=circularaccbufferx[x]; //running total of accelerations used to derive base or average
      acc_calibrationy+=circularaccbuffery[x];
      acc_calibrationz+=circularaccbufferz[x];

      if (circularaccbufferx[x]>xaxispeak)  xaxispeak=circularaccbufferx[x];
      if (circularaccbufferx[x]<xaxisvalley) xaxisvalley=circularaccbufferx[x];
      if (circularaccbuffery[x]>yaxispeak) yaxispeak=circularaccbuffery[x];
      if (circularaccbuffery[x]<yaxisvalley) yaxisvalley=circularaccbuffery[x];
      if (circularaccbufferz[x]>zaxispeak) zaxispeak=circularaccbufferz[x];
      if (circularaccbufferz[x]<zaxisvalley)  zaxisvalley=circularaccbufferz[x]; 
    } 

    xaxispeak=xaxispeak-int(acc_calibrationx/100);
    yaxispeak=yaxispeak-int(acc_calibrationy/100);
    zaxispeak=zaxispeak-int(acc_calibrationz/100);

    xaxisvalley=xaxisvalley-int(acc_calibrationx/100);
    yaxisvalley=yaxisvalley-int(acc_calibrationy/100);
    zaxisvalley=zaxisvalley-int(acc_calibrationz/100);

    if (consolemidimode==3)  //debug console outputs
    {
      int indexer=0; 
      for (int p=0; p<99; p++)
      {
        if (p==50) 
        {
          Serial.println("");
          Serial.println("hit point");
        }
        Serial.println("");
        Serial.print(p);
        Serial.print(" x:");
        Serial.print(circularaccbufferx[circbuffpointer+indexer]-int(acc_calibrationx/100));
        Serial.print(" ~y:");
        Serial.print(circularaccbuffery[circbuffpointer+indexer]-int(acc_calibrationy/100));
        Serial.print(" z:");
        Serial.print(circularaccbufferz[circbuffpointer+indexer]-int(acc_calibrationz/100));
        indexer++;
        if (indexer+circbuffpointer>99) indexer=-circbuffpointer;  
      } 


      Serial.println ("");   
      Serial.print (" zaxis peak:");
      Serial.print (zaxispeak);
      Serial.print (" xaxis peak:");
      Serial.print (xaxispeak);
      Serial.print (" yaxis peak:");
      Serial.println (yaxispeak);

      Serial.print (" zaxis valley:");
      Serial.print (zaxisvalley);
      Serial.print (" xaxis valley:");
      Serial.print (xaxisvalley);
      Serial.print (" yaxis valley:");
      Serial.println (yaxisvalley);

    } 


    for (int x=0; x<9; x++) //load up pending all notes with volume numbers
    {
      if (padmode[x]==-1) 
      {   
        if ((x==6) || (x==5)|| (x==2)|| (x==1)) //top of Kyub
        {
          padvolume[x]=-zaxisvalley;
          padmode[x]=2;    
        }

        if ((x==4)) //side of Kyub
        {
          padvolume[x]=xaxispeak; 
          padmode[x]=2;
        }

        if ((x==3) )
        {
          padvolume[x]=-xaxisvalley;
          padmode[x]=2;
        }

        if ((x==0) )
        {
          padvolume[x]=-yaxisvalley;
          padmode[x]=2;
        }

        if ((x==7)|| (x==8) )
        {
          padvolume[x]=yaxispeak;
          padmode[x]=2;
        }



      }
    } 
  }//end of triggerpoint=50


  //play notes *****************************************************************
  for (int x=0; x<9; x++)
  {
    if (padmode[x]==2)
    {
      notevolume=int((padvolume[x])*10);  //??room for improviment--mapping of accel to volume
      if (notevolume>127) notevolume=127;
      if (notevolume<10) notevolume=10;//was 10
      //notevolume=100;  //for testing
      padmode[x]=-2;  //-2 is ready for note off

      if (x==0)
      {
        pitch=note0;
        padlastchannel[x]=channel;
        padlastnote[x]=note0;
        noteOn(channel, pitch, notevolume); 
      }

      if (x==1)
      {
        pitch=note1;
        padlastchannel[x]=channel;
        padlastnote[x]=note1;
        noteOn(channel, pitch, notevolume); 
      }

      if (x==2)
      {
        pitch=note2;
        padlastchannel[x]=channel;
        padlastnote[x]=note2;
        noteOn(channel, pitch, notevolume); 
      }

      if (x==3)
      {
        pitch=note3;
        padlastchannel[x]=channel;
        padlastnote[x]=note3;
        noteOn(channel, pitch, notevolume);  
      }

      if (x==4)
      {
        pitch=note4;
        padlastchannel[x]=channel;
        padlastnote[x]=note4;
        noteOn(channel, pitch, notevolume); 
      }


      if (x==5)
      {
        pitch=note5;
        padlastchannel[x]=channel;
        padlastnote[x]=note5;
        noteOn(channel, pitch, notevolume); 
      }

      if (x==6)
      {
        pitch=note6;
        padlastchannel[x]=channel;
        padlastnote[x]=note6;
        noteOn(channel, pitch, notevolume); 
      }

      if (x==7)
      {
        pitch=note7;
        padlastchannel[x]=channel;
        padlastnote[x]=note7;
        noteOn(channel, pitch, notevolume);  
      }

      if (x==8)
      {
        pitch=note8;
        padlastchannel[x]=channel;
        padlastnote[x]=note8;
        noteOn(channel, pitch, notevolume);  
      }
    }
  }


  //turn off notes **************************************************
  for (int x=0; x<9; x++)
  {
    if ((padstate[x]==0) && (padmode[x]==-2)&& (micros()-padlasttime[x]>min_note_duration)) //need reset
    {
      padmode[x]=0;
      //digitalWrite(LEDpin, LOW);

      if (x==0)
      {
        pitch=padlastnote[x];
        channel=padlastchannel[x];
        noteOn(channel, pitch, volume);  //notevolume);
      }

      if (x==1)
      {
        pitch=padlastnote[x];
        channel=padlastchannel[x];
        noteOn(channel, pitch, 0);  //notevolume);
      }

      if (x==2)
      {
        pitch=padlastnote[x];
        channel=padlastchannel[x];
        noteOn(channel, pitch, 0);  //notevolume);
      }

      if (x==3)
      {
        pitch=padlastnote[x];
        channel=padlastchannel[x];
        noteOn(channel, pitch, 0);  //notevolume);
      }

      if (x==4)
      {
        pitch=padlastnote[x];
        channel=padlastchannel[x];
        noteOn(channel, pitch, 0);  //notevolume);
      }

      if (x==5)
      {
        pitch=padlastnote[x];
        channel=padlastchannel[x];
        noteOn(channel, pitch, 0);  //notevolume);
      }


      if (x==6)
      {
        pitch=padlastnote[x];
        channel=padlastchannel[x];
        noteOn(channel, pitch, 0);  //notevolume);
      }

      if (x==7)
      {
        pitch=padlastnote[x];
        channel=padlastchannel[x];
        noteOn(channel, pitch, 0);  //notevolume);
      }

      if (x==8)
      {
        pitch=padlastnote[x];
        channel=padlastchannel[x];
        noteOn(channel, pitch, 0);  //notevolume);
      }
    }
  }

  if ((micros()/1000000>next) &&  (consolemidimode==0))
    // wait a second so as not to send massive amounts of data
  {
    next++;
    for (int x=0; x<11; x++)  //prints out all charge times
    {
      Serial.print(" pin:");
      Serial.print(x);
      Serial.print("=");
      Serial.print(chargetime[x]-cap_calibration[x]);
    }
    Serial.println("");
    Serial.print ("padvol#1:  ");
    Serial.print (padmode[1]);
    Serial.print ("-xvalley: ");
    Serial.print(-xaxisvalley); 
    Serial.println("");
  } 

}//main loop












































































