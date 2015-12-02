/*  Animal tracker firmware final with time caliberation
 Ver4.2
 2014.02.08 
 */
#include <TinyGPS++.h>//gps libarry
#include <avr/wdt.h>//watchdog for count time
#include <avr/sleep.h>//save power
#include <SoftwareSerial.h>//for gsm UART
#include <avr/power.h>//save power
#include <TimerOne.h>//1 second timmer
#include <EEPROM.h>//for external eeprom
#include <OneWire.h>//for temperature sensor
#include <DallasTemperature.h>//for tempture reading
#include <WSWire.h>;//for external eeprom
#define  ONE_WIRE_BUS     PB6//temperature sensor pin
#define  TrackerID        12345
#define  Max_data_store   2340
//#define  Max_GPStime_cunt 300  //mark by ciou
//#define  Max_GSMtime_cunt 100  //mark by ciou
//#define  EEPROM_size 2340//  65535/28=2340
#define  Min_GSM_signal 3
#define  Powt 20000  //20 second
#define  GPowt 10000//10second
#define  Gsm_retry 6//cant get gsm signal or signal too low,reg same
#define  Gsm_sms_retry 3//cant send sms
#define  gps_hold_voltage 3850
#define  gsm_hold_voltage 3850

#define  wait_UTC 180  //10  // 240
#define  wait_location 180  //10 // 240
#define  wait_height 60  //10  // 60//1129
#define eeprom_retry 6

#define WD_out 200;

unsigned char data_send_count = 6;

unsigned char sv_counter = 0 ;
unsigned char sv_cycle = 38 ;  //sv detect cycle = 38*8 = 304 seconds
unsigned int  charge_vol = 4300;
unsigned int  high_vol = 4100;
unsigned char WD_counter = WD_out;
#define  gps_reset_time 2


#define FASTADC 1

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

///////////////////////////////////////////////////////////////////variable declaration
SoftwareSerial gsm_m(6,7);//RX,TX
unsigned char led = A2;
unsigned char GPS_SW = A3;
unsigned char GSM_SW = 5;
//unsigned char sensor_SW = PB7;
unsigned char battery_volt = A7;
unsigned char sv_sw = 10;
unsigned char WDInput = PB7;
//////////////////////////////////////////////////////////////////
int adcsra_save; //added by Linkai

unsigned int data_cunt = 0 ;//total data from start
unsigned int eeprom_pointer = 0 ;//where to write next data in eeprom
unsigned int data_offset=0;  //how many data need to be sent

volatile unsigned char syst_year=0;
volatile unsigned char syst_month=10;
volatile unsigned char syst_date=31;

volatile unsigned char syst_hour=0;
volatile unsigned char syst_min=0;

static unsigned char monthdays[12]={
  31,28,31,30,31,30,31,31,30,31,30,31};

volatile unsigned long sys_msec = 0 ;

volatile unsigned int gps_timer = 0 ;
//volatile unsigned int gsm_timer = 0 ;

volatile unsigned int timeout_timer = 0 ;


unsigned int gps_cycle = 10;  // 4 hrs 10 min = 4x60+10=250  //defult gps cycle is 1 day (1440 min) 
//unsigned int gsm_cycle = 1380;   // 23 hrs = 23*60 = 1380 //defult gsm cycle is 7 day (10080 min)

//unsigned int data_sent = 0;
boolean gps_flag = false;  // modify by Linkai, initial test
//boolean gsm_flag = true;  // modify by Linkai, initial test

//char* Number1="+886955421937";    //server phone number
//char* Number1="+886963790166";
  char* Number1="+886982049158";
//char* Number1="+886963232450";
boolean gpsonce=false;
//boolean gsmonce=false;
boolean gpssuss=false;

boolean gsmsuss=false;
//int sms_space=100;

//String send_sms_content="";
struct GPSdata
{                  //declar GPS data structure
  unsigned int index;

  unsigned char gpsyear;
  unsigned char gpsmonth;
  unsigned char gpsday;

  unsigned char gpshour;
  unsigned char gpsmin;
  unsigned char gpssec;

  unsigned long latitude;
  unsigned long longitude;
  
  unsigned int temperature;
  unsigned char datavaild; 
  unsigned int battery;
  unsigned long height;
  unsigned int hdop_value;
  unsigned int average_vab_count;
  unsigned char lat_dir;
  unsigned char lon_dir;
} 
;
volatile GPSdata tamp = {
  0, 0,0,0, 0,0,0, 0,0,0,0,0,0,0,0,'N','E'};    //declar GPS data buffer

double average_vab=0;
unsigned int vab_hour=0;
unsigned int vab_counter=0;


//TinyGPSPlus gps;
/////////////////////////////////////////////////////////////////////////////////////////////WDT interrupt
boolean time_set=false;//time not set 21
boolean time_syn=false;// time not sync 21
volatile unsigned int msec_value=8271;//


volatile  unsigned long notsyncounter=0;
unsigned int pre_count=0;

unsigned int countdayfrom2000(unsigned char inyear,unsigned char inmonth,unsigned char inday)
{
  unsigned int days=0;
  days+=inyear*365;
  for(unsigned char i=0;i<inyear;i++)
  {
    if(leapyear(i))
    {
      days++;
    }
  }


  for(int i=1;i<inmonth;i++)
  {
    if((i==2) && leapyear(inyear))
    {
      days+=29;
    }
    else
    {
      days+=monthdays[i-1];
    }  
  }

  days+=(inday-1);  
  return days;
}

boolean leapyear(unsigned char inyear)
{
  if((2000+inyear)%4==0)
  {
    if((2000+inyear)%100==0)
    {
      if((2000+inyear)%400==0)
      {
        return true;
      }
      else
      {
        return false;
      }  
    }
    else
    {
      return true;
    }  
  }
  else
  {
    return false;
  }  
}

boolean datebig(unsigned char inyear1,unsigned char inmonth1,unsigned char inday1,unsigned char inyear2,unsigned char inmonth2,unsigned char inday2)
{
  if(countdayfrom2000(inyear1,inmonth1,inday1)>countdayfrom2000(inyear2,inmonth2,inday2))
  {
    return true;
  }
  else
  {
    return false;
  }  
}

boolean dateequal(unsigned char inyear1,unsigned char inmonth1,unsigned char inday1,unsigned char inyear2,unsigned char inmonth2,unsigned char inday2)
{
  if(countdayfrom2000(inyear1,inmonth1,inday1)==countdayfrom2000(inyear2,inmonth2,inday2))
  {
    return true;
  }
  else
  {
    return false;
  }  
}

boolean timebigequal(unsigned char inyear1,unsigned char inmonth1,unsigned char inday1,unsigned char inhour1,unsigned char inmin1,unsigned char inyear2,unsigned char inmonth2,unsigned char inday2,unsigned char inhour2,unsigned char inmin2)
{
  if(datebig(inyear1,inmonth1,inday1,inyear2,inmonth2,inday2))
  {
    return true;
  }
  else
  {
    if(dateequal(inyear1,inmonth1,inday1,inyear2,inmonth2,inday2)) 
    {
      if((inhour1*60+inmin1)>=(inhour1*60+inmin1))
      {
        return true;
      }
      else
      {
        return false;
      }  
    }
    else
    {
      return false;
    } 

  }  
}

unsigned int date_minus(unsigned char inyear1,unsigned char inmonth1,unsigned char inday1,unsigned char inyear2,unsigned char inmonth2,unsigned char inday2)
{
  unsigned int days=0;  
  if(datebig(inyear1,inmonth1,inday1,inyear2,inmonth2,inday2))
  {
    days=countdayfrom2000(inyear1,inmonth1,inday1)-countdayfrom2000(inyear2,inmonth2,inday2);
  }
  else
  {
    days=countdayfrom2000(inyear2,inmonth2,inday2)-countdayfrom2000(inyear1,inmonth1,inday1);
  }
  return days;
}

long time_minus(unsigned char inyear1,unsigned char inmonth1,unsigned char inday1,unsigned char inhour1,unsigned char inmin1,unsigned char inyear2,unsigned char inmonth2,unsigned char inday2,unsigned char inhour2,unsigned char inmin2)
{
  if(timebigequal(inyear1,inmonth1,inday1,inhour1,inmin1,inyear2,inmonth2,inday2,inhour2,inmin2))
  {
    return (date_minus(inyear1,inmonth1,inday1,inyear2,inmonth2,inday2)*24+inhour1-inhour2)*60+(inmin1-inmin2);
  }
  else
  {
    return (date_minus(inyear2,inmonth2,inday2,inyear1,inmonth1,inday1)*24+inhour2-inhour1)*60+(inmin2-inmin1);
  }
}

ISR( WDT_vect ) 
{
  power_all_enable();  
  //digitalWrite(led,HIGH); //for debug  mark by Linkai
  //  sleep_cunt++;
  sys_msec=sys_msec+msec_value;  //add time
  if(sys_msec >= 60000)
  { 
    if(time_set)
    {
      notsyncounter++;
    }

    if(pre_count>0)//min cal 
    {
      pre_count--;
    }
    else
    {
      gps_timer++;
    }
    //gsm_timer++;
    syst_min++;
    sys_msec = sys_msec % 60000; 
  }

  if(syst_min>=60)
  {
    average_vab=(average_vab*vab_hour+vab_counter)/(vab_hour+1);
    vab_hour++;
    vab_counter=0;

    syst_hour++;
    syst_min=syst_min%60;
  }

  if(syst_hour>=24)
  {
    syst_date++;
    syst_hour=syst_hour%24;
  }

  if(syst_date>monthdays[syst_month-1] || (leapyear(syst_year)&&(syst_month==2)&&(syst_date>29)))
  {
    if((syst_month==2) && leapyear(14+syst_year))
    {
      if(syst_date>29)
      {
        syst_date=1;
        syst_month++;  

      }
    }  
    else
    {
      syst_date=1;
      syst_month++;  

    }
  }

  if(syst_month>12)
  {
    syst_year++;
    syst_month=1;
  }

  Serial.print(F("mv : "));  //Mark by Linkai
  Serial.println(msec_value,DEC);   //Mark by Linkai
  Serial.print(F("Free:"));   //Mark by Linkai
  Serial.println(free_ram());    // Mark by Linkai

  if((gps_timer == gps_cycle) && (!gpsonce)) //gps cycle mode  //see if gps should turn on now ,gsmonce is the variable for checking the data has been sent
  {
    gps_flag = true;
    //Serial.println(F("gps"));   //Mark by Linkai
  }

  if((gps_timer >= gps_reset_time) && gpssuss ) //if 1min pass and gps recoding sussessfully
  {
    gpsonce=false;
    gpssuss=false;//reset
    //Serial.println(F("gps r"));   //Mark by Linkai
  }

  sv_counter++;                        //avoid over charge
  if(sv_counter >= sv_cycle){
    int battvol = get_voltage(); 
    sv_counter = 0 ;
    if(battvol >= charge_vol)
      digitalWrite(sv_sw,HIGH);
    else if(battvol < charge_vol && battvol >= high_vol){
      digitalWrite(sv_sw,LOW);
      gps_cycle = 30 ;    
    }
    else{
      digitalWrite(sv_sw,LOW);
      gps_cycle = 240 ;  
    }
  }
  
  if(WD_counter != 0){
    WD_counter--;
    digitalWrite(WDInput,HIGH);
    //delay(1);
    digitalWrite(WDInput,LOW);
  }
    


}

/////////////////////////////////////////////////////////////////////////////////////////////timer1 interrupt
void vab_count()
{
  //delay(20);//non-state
  vab_counter++;
}

void timer_1s()
{
  timeout_timer++;
  Serial.print(F("wt: "));
  Serial.println(timeout_timer,DEC);
}
////////////////////////////////////////////////////////////////////////////////////////////
int free_ram()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
////////////////////////////////////////////////////////////////////////////////////////////
void get_GPS_data(int batteryvol)
{ 
  tamp.battery=0;
  tamp.battery = batteryvol;    
  tamp.datavaild=0;//reset
  //Serial.overflow();
  //Serial.flush();
  
  if(batteryvol>gps_hold_voltage)//battery low,skip this time
  {
    digitalWrite(GPS_SW,HIGH);
    delay(GPowt);
    Timer1.attachInterrupt(timer_1s);
    timeout_timer=0;
    if(get_GPS_datetime())//mod by ciou
    {
      if(get_GPS_location())
      {
        tamp.datavaild+=0x08;
        timeout_timer=0;
        if(get_GPS_height())
        {
          tamp.datavaild+=0x04;
        }

      }


    }

    Timer1.detachInterrupt();
    Serial.println(F("gend"));
    digitalWrite(GPS_SW,LOW);
    Serial.flush();  
    int gsm_retry_count=0;//21
    boolean get_signal=false;
    delay(10000);           //150918   by shrimp
    digitalWrite(GSM_SW,HIGH);//21turn on
    gsm_initial();//21
    while(gsm_retry_count<Gsm_retry&&!get_signal)
    {
      //wait
      Serial.println(F("wgsm"));
      int signal = gsm_signal();
      Serial.print(F("Gs: "));
      Serial.println(signal,DEC); 
      if(signal!=-1)
      { 
        get_signal=true;
        tamp.temperature = signal;
      }

    }
    digitalWrite(GSM_SW,LOW);
  }
  else
  {
      tamp.gpsyear=syst_year;
    tamp.gpsmonth=syst_month;
    tamp.gpsday=syst_date;
    tamp.gpshour = syst_hour;
    tamp.gpsmin =  syst_min;
    tamp.gpssec =  sys_msec/1000;
    time_syn=false;
  }

  if(time_syn)
  {
    tamp.datavaild+=0x02;
  }

  if(time_set)
  {  
    tamp.datavaild+=0x01;
  }
  
  if(vab_hour==0)
  {
    tamp.average_vab_count=vab_counter;
  }
  else  
  {
    tamp.average_vab_count=(int)average_vab;
    average_vab=0;
    vab_hour=0; 
  }
  tamp.index = data_cunt + 1;
  //tamp.temperature = get_temp();

  Serial.println(F("dend"));
  save_data(); 
  //Serial.println(F("send"));

}

///////////////////////////////////////////////////////////////////////////////////////////

boolean get_GPS_location()
{
  TinyGPSPlus gps;
  unsigned char start[7]={
    '$','G','P','G','S','A',','                                                    };
  int index=0;
  int dotc=0;
  boolean record=false;
  boolean vaild=false;  
  Serial.flush();
  ////////////////////////////wait location  (180s timeout)
  while(!(gps.location.isUpdated() && gps.location.isValid() && gps.hdop.isValid() && gps.hdop.isUpdated() && vaild) && timeout_timer<=wait_location)
  {
    while(Serial.available() > 0)
    {
      char CharRead=Serial.read();
      if(dotc==2)
      {
        Serial.write(CharRead);  
        if(CharRead=='2' || CharRead=='3')
        {
          vaild=true;
          Serial.println(F("locv"));
          record=false;
          dotc=0;
        }
        else
        {
          record=false;
          dotc=0;
        }  
      }

      if (CharRead==start[index])
      {
        if(index == 6)
        {
          record=true; 
        } 
        index++;
      }
      else
      {
        index = 0;
      }

      if(record && CharRead==',')
      {
        dotc++;
      }

      if(vaild)
      {  
        gps.encode(CharRead);
      }
    }                      
  }

  if(gps.location.isValid() && gps.location.isUpdated() && gps.hdop.isValid() && gps.hdop.isUpdated() && vaild)
  { 
    //////////////////////////////record location
    Serial.print(F("Loc"));
    tamp.latitude = gps.location.lat()*10000000;
    tamp.longitude =gps.location.lng()*10000000;
    tamp.hdop_value=gps.hdop.value();

    tamp.lat_dir=gps.location.rawLat().negative ? 'S' : 'N';
    tamp.lon_dir=gps.location.rawLng().negative ? 'W' : 'E';
    return true; 
  }
  else
  {
    tamp.latitude = 0;
    tamp.longitude = 0;
    return false;        
  }
  return false;         
}

boolean get_GPS_height()
{
  TinyGPSPlus gps;
  unsigned char start[7]={
    '$','G','P','G','S','A',','                                                    };
  int index=0;
  int dotc=0;
  boolean record=false;
  boolean vaild=false;
  Serial.flush();  
  ////////////////////////////wait alitude  (60s timeout)
  while(!(gps.altitude.isUpdated() && gps.altitude.isValid() && vaild) && timeout_timer<=wait_height)
  {
    while(Serial.available() > 0)
    {
      char CharRead=Serial.read();
      //Serial.write(CharRead);
      if(dotc==2)
      {
        Serial.write(CharRead);  
        if(CharRead=='3')
        {
          vaild=true;
          Serial.println(F("hev"));
          record=false;
          dotc=0;
        }
        else
        {
          record=false;
          dotc=0;
        }  
      }

      if (CharRead==start[index])
      {
        if(index == 6)
        {
          record=true; 
        } 
        index++;
      }
      else
      {
        index = 0;
      }

      if(record && CharRead==',')
      {
        dotc++;
      }

      if(vaild)
      {  
        gps.encode(CharRead);
      }
    }                      
  }

  if(gps.altitude.isValid() && gps.altitude.isUpdated()&& vaild)
  { 
    //////////////////////////////record alitude
    tamp.height = gps.altitude.meters();
    return true; 
  }
  else
  {
    tamp.height=0;
    return false;        
  }
  return false;  
}

boolean get_GPS_datetime()
{
  TinyGPSPlus gps;
  unsigned char start[7]={
    '$','G','P','G','S','A',','                            };
  int index=0;
  int dotc=0;
  boolean record;
  boolean vaild;  
  record=false;
  vaild=false;
  Serial.flush();
  //timeout_timer=0;
  while(!(gps.date.isValid() && gps.date.isUpdated() && gps.time.isValid() && gps.time.isUpdated() && vaild) && timeout_timer<=wait_UTC)
  {    
    while(Serial.available() > 0)
    {
      char CharRead=Serial.read();
      //Serial.write(CharRead);

      if(dotc==2)
      {
        Serial.write(CharRead);  
        if(CharRead=='2' || CharRead=='3')
        {
          vaild=true;
          Serial.println(F("tv"));
          record=false;
          dotc=0;
        }
        else
        {
          record=false;
          dotc=0;
        }  
      }

      if (CharRead==start[index])
      {
        if(index == 6)
        {
          record=true; 
        } 
        index++;
      }
      else
      {
        index = 0;
      }

      if(record && CharRead==',')
      {
        dotc++;
      }
      if(vaild)
      {  
        gps.encode(CharRead);
      }
    }
  }

  if(gps.date.isValid() && gps.date.isUpdated() && gps.time.isValid() && gps.time.isUpdated() && vaild)
  { 

    if(time_set)
    {
      unsigned int minustemp= time_minus(gps.date.year()-2000,gps.date.month(),gps.date.day(),gps.time.hour(),gps.time.minute(),syst_year+14,syst_month,syst_date,syst_hour,syst_min); 


      if(timebigequal(gps.date.year()-2000,gps.date.month(),gps.date.day(),gps.time.hour(),gps.time.minute(),syst_year+14,syst_month,syst_date,syst_hour,syst_min))
      {
        unsigned int msectemp=0;
        msectemp=msec_value+(int)((msec_value*minustemp)/notsyncounter);  
        //msectemp=msec_value*(1+msectemp);
        Serial.print(F("m:"));
        Serial.println(msectemp,DEC); 
        /*if((msectemp>6000) && (msectemp<10000))
         {
         msec_value=msectemp;
         Serial.print(F("neva:"));
         Serial.println(msec_value,DEC);
         }*/
        Serial.print(F("g:"));
        Serial.println(gps_timer,DEC);
        gps_timer= (gps_timer+minustemp)%gps_cycle;
        //gps_timer= (gps_timer+minustemp*msec_value/msectemp)%gps_cycle;
        Serial.print(F("ag:"));
        Serial.println(gps_timer,DEC); 
      }
      else
      {
        unsigned int msectemp=0;
        msectemp=msec_value-(int)((msec_value*minustemp)/(notsyncounter));  
        //msectemp=msec_value*(1+msectemp);
        Serial.print(F("m:"));
        Serial.println(msectemp,DEC); 
        /*if(msectemp>6000 && msectemp<10000)
         {
         msec_value=msectemp;
         Serial.print(F("neva:"));
         Serial.println(msec_value,DEC);
         
         } */
        Serial.print(F("p:"));
        Serial.println(pre_count,DEC);  
        pre_count= minustemp%gps_cycle;
        //pre_count= (minustemp*msec_value/msectemp)%gps_cycle;
        Serial.print(F("ap:"));
        Serial.println(pre_count,DEC);
      }


    }

    Serial.println(F("dat"));
    syst_date =gps.date.day();
    syst_month=gps.date.month();
    syst_year=gps.date.year()-2014;


    //tamp.gpsyear=gps.date.year()-2014;
    //tamp.gpsmonth=gps.date.month();
    //tamp.gpsday=gps.date.day();

    Serial.println(F("ti"));
    sys_msec = gps.time.second()* 1000+gps.time.centisecond()*1;
    syst_min = gps.time.minute();
    syst_hour= gps.time.hour();
    /*Serial.print(gps.time.hour(),DEC);
     Serial.print(F(" : "));
     Serial.print(gps.time.minute(),DEC);
     Serial.print(F(" : "));
     Serial.println(gps.time.second(),DEC);*/
    //tamp.gpshour = gps.time.hour();
    //tamp.gpsmin = gps.time.minute();
    //tamp.gpssec = gps.time.second();

    tamp.gpsyear=syst_year;
    tamp.gpsmonth=syst_month;
    tamp.gpsday=syst_date;
    tamp.gpshour = syst_hour;
    tamp.gpsmin =  syst_min;
    tamp.gpssec =  sys_msec/1000;
    time_set=true;
    time_syn=true;
    notsyncounter=0;
    return true;    
  }
  else
  {
    tamp.gpsyear=syst_year;
    tamp.gpsmonth=syst_month;
    tamp.gpsday=syst_date;
    tamp.gpshour = syst_hour;
    tamp.gpsmin =  syst_min;
    tamp.gpssec =  sys_msec/1000;

    time_syn=false;
    return false;
  }
  return false;  
}
///////////////////////////////////////////////////////////////////////////////////////////
/*unsigned int get_temp()
 {
 double temp;
 unsigned int tempout;
 digitalWrite(sensor_SW,HIGH);
 //sensors.begin();
 // sensors.setResolution(12);
 //  sensors.requestTemperatures();  
 //temp = sensors.getTempCByIndex(0);
 tempout = temp*100; 
 digitalWrite(sensor_SW,LOW);  
 return tempout;
 }*/
////////////////////////////////////////////////////////////////////////////////////////////

unsigned int get_voltage()
{
  unsigned int volt  ;
  //double vv ;
  volt=readVcc();///1000;//get very accurry voltage inside core 3.329
  return volt;
}

long readVcc() {

  //Start up ADC add by Linkai

  //Serial.print(F("ADCSRA_read="));
  //Serial.println(ADCSRA,DEC); 
  ADCSRA = adcsra_save;
  //Serial.print(F("adcsra_save="));
  //Serial.println(adcsra_save,DEC); 
  //Serial.print(F("ADCSRA="));
  // Serial.println(ADCSRA,DEC); 

  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif  

  delay(1000); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  //Serial.print(F(" VCC "));
  //Serial.print(high,DEC);
  //Serial.print(F(" : "));
  //Serial.print(low,DEC);
  // Serial.print(F(" : "));
  //Serial.print(result,DEC);  



  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000

  //Serial.print(F(" : "));
  Serial.println(result,DEC);  
  ADCSRA=0;

  return result; // Vcc in millivolts
}

/////////////////////////////////////////////////////////////////////////////////////////////save data from buffer to eeprom
void save_data_index()
{
  char write_retry=0;
  boolean write_suss=false;
  boolean data_check;
  while((!write_suss) && write_retry<eeprom_retry)
  {
    data_check=true;  
    int temp1;
    //long temp2;
    //EEPROM.write(0x00, 0x01);
    //data_check=data_check&(EEPROM.read(0x00)==0x01);

    EEPROM.write(0x01, (data_offset/200)&0x01);
    EEPROM.write(0x02, (data_offset%200)&0x01);
    temp1=EEPROM.read(0x01)*200+EEPROM.read(0x02);
    data_check=data_check&(temp1==data_offset);

    EEPROM.write(0x03,(eeprom_pointer/200)&0x01);
    EEPROM.write(0x04,(eeprom_pointer%200)&0x01);
    temp1=EEPROM.read(0x03)*200+EEPROM.read(0x04);
    data_check=data_check&(temp1==eeprom_pointer);

    EEPROM.write(0x05,(data_cunt/200)&0x01); 
    EEPROM.write(0x06,(data_cunt%200)&0x01); 
    temp1=EEPROM.read(0x05)*200+EEPROM.read(0x06);
    data_check=data_check&(temp1==data_cunt);
    //EEPROMwrite(0 ,);
    //EEPROMwrite(1 ,tamp.index);
    //temp1=EEPROMread()<<8+EEPROMread(offset+1);
    //data_check=data_check&(temp1==tamp.index);
    if(data_check)
      {
      EEPROM.write(0x00, 0x01);
      data_check=data_check&(EEPROM.read(0x00)==0x01);
      }
    else
      {
      EEPROM.write(0x00, 0x00);
      data_check=data_check&(EEPROM.read(0x00)!=0x01);     
      }  
  
    if(data_check)
    {
      
      write_suss=true;
    }
    else
    {
      write_retry++;  
    }  
  }
}

void load_data_index()
{
  char read_retry=0;
  boolean read_suss=false;
  boolean data_check;
  while((!read_suss) && read_retry<eeprom_retry)
  {
    //data_check=true;  \\\\\\\\\\\\\\\\\\\\\\\\
    //int temp1;
    unsigned int temp2,temp3,temp4;

    int temp1=EEPROM.read(0x00);
    data_check=(temp1==EEPROM.read(0x00));


    if(data_check)
    {
      if(temp1==0x01)
      {
        char load1,load2;
        load1=EEPROM.read(0x01);
        load2=EEPROM.read(0x02);
        temp2=load1*200+load2;
        data_check=data_check&(temp2==EEPROM.read(0x01)*200+EEPROM.read(0x02))&(load2<200)&(load1<255);
        
        load1=EEPROM.read(0x03);
        load2=EEPROM.read(0x04);
        temp3=load1*200+load2;
        data_check=data_check&(temp3==EEPROM.read(0x03)*200+EEPROM.read(0x04))&(load2<200)&(load1<255);
        
        load1=EEPROM.read(0x05);
        load2=EEPROM.read(0x06);
        temp4=load1*200+load2;
        data_check=data_check&(temp4==EEPROM.read(0x05)*200+EEPROM.read(0x06))&(load2<200)&(load1<255);
      }
    }

    if(data_check)
    {
    if(temp1==0x01)
      {
      data_offset=temp2;
      eeprom_pointer=temp3;
      data_cunt=temp4;
      }
      else
      {
      data_offset=0;
      eeprom_pointer=0;
      data_cunt=0;
      }
      read_suss=true;
    }
    else
    {
      read_retry++;
    }  
  }      
}

void save_data()
{

  unsigned long offset;
  char write_retry=0;
  boolean write_suss=false;
  boolean data_check;
  while((!write_suss) && write_retry<eeprom_retry)
  {
    data_check=true;  
    offset = 28 * eeprom_pointer;  
    int temp1;
    long temp2;
    digitalWrite(8,HIGH);
    delay(100); 
    EEPROMwrite(offset+0 ,tamp.index>>8);
    EEPROMwrite(offset+1 ,tamp.index);
    temp1=(EEPROMread(offset)<<8)+EEPROMread(offset+1);
    data_check=data_check&(temp1==tamp.index);

    EEPROMwrite(offset+2 ,tamp.gpssec);
    data_check=data_check&((EEPROMread(offset+2))==tamp.gpssec);

    EEPROMwrite(offset+3 ,tamp.gpsmin);
    data_check=data_check&((EEPROMread(offset+3))==tamp.gpsmin);

    EEPROMwrite(offset+4 ,tamp.gpshour);
    data_check=data_check&((EEPROMread(offset+4))==tamp.gpshour);

    EEPROMwrite(offset+5 ,tamp.gpsday);
    data_check=data_check&((EEPROMread(offset+5))==tamp.gpsday);

    EEPROMwrite(offset+6 ,tamp.gpsmonth);
    data_check=data_check&((EEPROMread(offset+6))==tamp.gpsmonth);

    EEPROMwrite(offset+7 ,tamp.gpsyear);
    data_check=data_check&((EEPROMread(offset+7))==tamp.gpsyear);

    EEPROMwrite(offset+8 ,tamp.latitude>>24);
    EEPROMwrite(offset+9 ,tamp.latitude>>16);
    EEPROMwrite(offset+10 ,tamp.latitude>>8);
    EEPROMwrite(offset+11,tamp.latitude);
    temp2=(EEPROMread(offset+8)<<24)+(EEPROMread(offset+9)<<16)+(EEPROMread(offset+10)<<8)+(EEPROMread(offset+11));
    data_check=data_check&(temp2==tamp.latitude);

    EEPROMwrite(offset+12,tamp.longitude>>24);
    EEPROMwrite(offset+13,tamp.longitude>>16);
    EEPROMwrite(offset+14,tamp.longitude>>8);
    EEPROMwrite(offset+15,tamp.longitude);
    temp2=(EEPROMread(offset+12)<<24)+(EEPROMread(offset+13)<<16)+(EEPROMread(offset+14)<<8)+(EEPROMread(offset+15));
    data_check=data_check&(temp2==tamp.longitude);

    EEPROMwrite(offset+16,tamp.temperature);
    data_check=data_check&((EEPROMread(offset+16))==tamp.temperature);

    EEPROMwrite(offset+17,tamp.height>>8);
    EEPROMwrite(offset+18,tamp.height);
    temp1=(EEPROMread(offset+17)<<8)+EEPROMread(offset+18);
    data_check=data_check&(temp1==tamp.height);

    EEPROMwrite(offset+19,tamp.datavaild);
    data_check=data_check&((EEPROMread(offset+19))==tamp.datavaild);

    EEPROMwrite(offset+20,tamp.hdop_value>>8);
    EEPROMwrite(offset+21,tamp.hdop_value);
    temp1=(EEPROMread(offset+20)<<8)+EEPROMread(offset+21);
    data_check=data_check&(temp1==tamp.hdop_value);

    EEPROMwrite(offset+22,tamp.average_vab_count>>8);
    EEPROMwrite(offset+23,tamp.average_vab_count);
    temp1=(EEPROMread(offset+22)<<8)+(EEPROMread(offset+23));
    data_check=data_check&(temp1==tamp.average_vab_count);  

    EEPROMwrite(offset+24,tamp.lat_dir);
    data_check=data_check&((EEPROMread(offset+24))==tamp.lat_dir);

    EEPROMwrite(offset+25,tamp.lon_dir);
    data_check=data_check&((EEPROMread(offset+25))==tamp.lon_dir);

    EEPROMwrite(offset+26,tamp.battery>>8);
    EEPROMwrite(offset+27,tamp.battery);
    temp1=(EEPROMread(offset+26)<<8)+(EEPROMread(offset+27));
    data_check=data_check&(temp1==tamp.battery); 
    digitalWrite(8,LOW);
    if(data_check)
    {
      write_suss=true;
    }
    else
    {
      write_retry++; 
     delay(6000); 
    }  

  }
  eeprom_pointer++;
  if(eeprom_pointer>=Max_data_store)
  {
    eeprom_pointer=0;
  }

  if(data_offset>=Max_data_store)
  {
    data_offset=Max_data_store;
  }
  else
  {
    data_offset++;
  }

  data_cunt++;
  save_data_index(); 
  //print_GPSbuffer();

 // clear_data();
}




////////////////////////////////////////////////////////////////////////////////////////load data from eeprom to buffer
void load_data(long indx)
{
  unsigned long offset = 0xffffffff;
  unsigned long a,b,c,d;

  char read_retry=0;
  boolean read_suss=false;
  boolean data_check;
  while((!read_suss) && read_retry<eeprom_retry)
  {
    data_check=true;  
    digitalWrite(8,HIGH);
    delay(100); 
    int temp1;
    long temp2;

    offset = indx * 28;
    tamp.index = (EEPROMread(offset+0)<<8)+(int)EEPROMread(offset+1);
    data_check=data_check&(tamp.index == ((EEPROMread(offset+0)<<8)+(int)EEPROMread(offset+1)));
    /*tamp.gpstime = (EEPROMread(offset+2)<<8)+EEPROMread(offset+3);
     a=EEPROMread(offset+4);
     b=EEPROMread(offset+5); 
     c=EEPROMread(offset+6);  
     tamp.gpsdate = (a<<16)+(b<<8)+c;*/
    tamp.gpssec=EEPROMread(offset+2);
    data_check=data_check&(tamp.gpssec==EEPROMread(offset+2));

    tamp.gpsmin=EEPROMread(offset+3);
    data_check=data_check&(tamp.gpsmin==EEPROMread(offset+3));

    tamp.gpshour=EEPROMread(offset+4);
    data_check=data_check&(tamp.gpshour==EEPROMread(offset+4));

    tamp.gpsday=EEPROMread(offset+5);
    data_check=data_check&(tamp.gpsday==EEPROMread(offset+5));

    tamp.gpsmonth=EEPROMread(offset+6);
    data_check=data_check&(tamp.gpsmonth==EEPROMread(offset+6));

    tamp.gpsyear=EEPROMread(offset+7);
    data_check=data_check&(tamp.gpsyear==EEPROMread(offset+7));


    a=EEPROMread(offset+8);
    b=EEPROMread(offset+9); 
    c=EEPROMread(offset+10);
    d=EEPROMread(offset+11); 
    tamp.latitude = (a<<24)+(b<<16)+(c<<8)+d;
    a=EEPROMread(offset+8);
    b=EEPROMread(offset+9); 
    c=EEPROMread(offset+10);
    d=EEPROMread(offset+11);   
    data_check=data_check&(tamp.latitude == (a<<24)+(b<<16)+(c<<8)+d);


    a=EEPROMread(offset+12);
    b=EEPROMread(offset+13); 
    c=EEPROMread(offset+14);
    d=EEPROMread(offset+15);   
    tamp.longitude = (a<<24)+(b<<16)+(c<<8)+d;
    a=EEPROMread(offset+12);
    b=EEPROMread(offset+13); 
    c=EEPROMread(offset+14);
    d=EEPROMread(offset+15); 
    data_check=data_check&(tamp.longitude == (a<<24)+(b<<16)+(c<<8)+d);

    tamp.temperature = EEPROMread(offset+16);
    data_check=data_check&(tamp.temperature == EEPROMread(offset+16));

    //tamp.battery = EEPROMread(offset+17);
    c=EEPROMread(offset+17);
    d=EEPROMread(offset+18);
    tamp.height = c<<8+d;
    c=EEPROMread(offset+17);
    d=EEPROMread(offset+18);
    data_check=data_check&(tamp.height == c<<8+d);

    tamp.datavaild=EEPROMread(offset+19);
    data_check=data_check&(tamp.datavaild==EEPROMread(offset+19));

    c=EEPROMread(offset+20);
    d=EEPROMread(offset+21);
    tamp.hdop_value=(c<<8)+d;
    c=EEPROMread(offset+20);
    d=EEPROMread(offset+21);  
    data_check=data_check&(tamp.hdop_value==(c<<8)+d);


    c=EEPROMread(offset+22);
    d=EEPROMread(offset+23);
    tamp.average_vab_count=(c<<8)+d;
    c=EEPROMread(offset+22);
    d=EEPROMread(offset+23);  
    data_check=data_check&(tamp.average_vab_count==(c<<8)+d);

    tamp.lat_dir=EEPROMread(offset+24);
    data_check=data_check&(tamp.lat_dir==EEPROMread(offset+24));

    tamp.lon_dir=EEPROMread(offset+25);
    data_check=data_check&(tamp.lon_dir==EEPROMread(offset+25));

    c=EEPROMread(offset+26);
    d=EEPROMread(offset+27);
    tamp.battery=(c<<8)+d;
    c=EEPROMread(offset+26);
    d=EEPROMread(offset+27);
    data_check=data_check&(tamp.battery==(c<<8)+d);
    digitalWrite(8,LOW);
    if(data_check)
    {
      read_suss=true;
    }
    else
    {
      read_retry++;
      delay(6000);
    }  
  }
}
//////////////////////////////////////////////////////////////////////////////////////////set all data in eeprom to 0x00
void clear_data()
{
  tamp.index = 0;

  tamp.gpshour = 0;
  tamp.gpsmin  = 0;
  tamp.gpssec = 0;

  tamp.gpsyear = 0;
  tamp.gpsmonth = 0;
  tamp.gpsday = 0;

  tamp.latitude = 0;
  tamp.longitude = 0;
  tamp.temperature = 0;
  tamp.height = 0;
  tamp.height=0;
  tamp.hdop_value=0;
  tamp.average_vab_count=0;
  tamp.lat_dir=0;
  tamp.lon_dir=0;
  tamp.battery=0;

}
/////////////////////////////////////////////////////////////////////////////////////////
/*void clear_eeprom()
 {
 //console.print(F("Start clear EEPROM......"));
 unsigned long i; 
 for(i=0;i<EEPROM_size;i++)
 {
 EEPROMwrite(i,0);
 }
 //console.println(F("done"));  
 }*/

/////////////////////////////////////////////////////////////////////////////////////////print current data in gps data buffer

/*void print_GPSbuffer()
 {
 Serial.print(F("CURRENT Data: "));
 Serial.println(data_offset);
 Serial.print(F("Data number: "));
 Serial.println(tamp.index);  
 Serial.print(F("location: "));
 Serial.print(tamp.latitude);
 Serial.print(F(" , "));
 Serial.println(tamp.longitude);    
 Serial.print(F("date: "));
 Serial.print(tamp.gpsyear);   
 Serial.print(tamp.gpsmonth);
 Serial.println(tamp.gpsday);   
 Serial.print(F("time: "));
 Serial.print(tamp.gpshour);
 Serial.print(tamp.gpsmin);
 Serial.println(tamp.gpssec);    
 Serial.print(F("temp: "));
 Serial.println(tamp.temperature);  
 Serial.print(F("battery: "));
 Serial.println(tamp.battery);
 Serial.print(F("height: "));
 Serial.println(tamp.height);
 
 Serial.print(F("hDOP: "));
 Serial.println(tamp.hdop_value);
 Serial.print(F("AVVA: "));
 Serial.println(tamp.average_vab_count); 
 }
 */

//////////////////////////////////////////////////////////////////////////////////////////////
/*gsm functions*/
void gsm_initial(void)/*V1.6*/
{
  boolean sets=false; 
  String tmps="";
  tmps.reserve(128); 
  //set text start
  gsm_m.flush();
  //tmps="";
  gsm_m.print("ATE0\r");
  Timer1.attachInterrupt(timer_1s);
  //Timer1.start();
  timeout_timer=0;
  while((timeout_timer<20)&&(!sets))
  {    
    while(gsm_m.available()>0) 
    {
      char cache=gsm_m.read();
      tmps+=cache; 
    }

    if(tmps.indexOf("OK")==-1)
    {
      if(tmps.indexOf("ERROR")!=-1)
      {
        sets=true;
        Serial.print(F("Gie"));
      }
    }
    else
    {
      Serial.print(F("Gis"));
      sets=true;
    } 
    delay(10);   
  }
  Timer1.detachInterrupt();
}


boolean gsm_available(int level)//V1.1
{
  if(level>Min_GSM_signal)
  {
    return true;
  }
  else
  {
    return false;
  }  
}

int gsm_signal(void)//V1.1
{
  String tmps="";
  tmps.reserve(128); 
  gsm_m.flush();
  gsm_m.print("AT+CSQ\r");    //Send the SMS in text mode

  //tmps="";
  Timer1.attachInterrupt(timer_1s);
  //Timer1.start();
  timeout_timer=0;
  while((timeout_timer<20))
  {  
    while(gsm_m.available()) 
    {
      char cache=gsm_m.read();
      tmps=tmps+cache;
    }

    if(tmps.indexOf("+CSQ:")!=-1)
    {
      int sl=0;
      int i=tmps.indexOf("+CSQ:")+6;
      while(tmps.charAt(i)!=',')
      {
        sl=sl*10+tmps.charAt(i)-48;
        if(i>tmps.indexOf("+CSQ:")+10)
        {
          Serial.println(F("Sro"));
          Timer1.detachInterrupt(); 
          return -1;
        }
        i++;  
      }

      if(sl==99)
      {
        Serial.println(F("Su"));
        Timer1.detachInterrupt(); 
        return -1;
      }
      else
      {
        Serial.println(F("Ss"));
        Timer1.detachInterrupt(); 
        return sl;
      } 
    }
    else
    {
      if(tmps.indexOf("ERROR")!=-1)
      {
        Serial.println(F("Se"));  
        Timer1.detachInterrupt();   
        return -1;
      }
    }
  }
  Serial.println(F("Sto")); 
  Timer1.detachInterrupt();   
  return -1;  

}

int gsm_registration_status(void)/*V1.2*/
{
  int startp; 
  String tmps="";
  tmps.reserve(128); 
  gsm_m.flush();
  gsm_m.print("AT+CREG?\r");
  delay(100);

  //tmps="";
  Timer1.attachInterrupt(timer_1s);
  timeout_timer=0;
  while(timeout_timer<10)
  {
    while(gsm_m.available()>0) 
    {
      char cache=gsm_m.read();
      tmps=tmps+cache;
    }
    //Serial.println(tmps);  
    startp=tmps.indexOf("+CREG:");//  
    if(startp!=-1)
    { 
      int i=tmps.indexOf(",",startp+7)+1;
      char tempr=tmps.charAt(i);
      if((tempr>47)&&(tempr<58))
      {
        Timer1.detachInterrupt();  
        return tempr-48;	  
      }
      else
      {
        Timer1.detachInterrupt();  
        return -1;
      }
    }
  }
  Timer1.detachInterrupt(); 
  return -1;  
}

boolean gsm_registration(int restatus)/*V1.1*/
{
  if(restatus==1)
  {
    return true;
  }
  else
  {
    return false;
  }  
}

boolean set_text_mode(int retry)//V 1.1
{
  String tmps="";
  tmps.reserve(128); 
  gsm_m.flush();
  gsm_m.print("AT+CMGF=1\r");    //Send the SMS in text mode
  //delay(10);
  //Serial.println(F("text mode set"));
  //tmps="";
  Timer1.attachInterrupt(timer_1s);
  //Timer1.start();
  timeout_timer=0;
  while(timeout_timer<retry)
  {
    while(gsm_m.available()>0) 
    {
      char cache=gsm_m.read();
      tmps=tmps+cache;
    }

    //Serial.print(tmps);  
    if(tmps.indexOf("OK")==-1)
    {

      //console.println(tmps);
      //Serial.println(F("not set:"));
      //Serial.print(tmps);   
      if(tmps.indexOf("ERROR")!=-1)
      {
        //console.print("ERROR");
        Serial.println(F("ste"));
        Timer1.detachInterrupt();
        //Timer1.stop();    
        return false;
      }   
    }
    else
    {
      Serial.println(F("sts"));
      Timer1.detachInterrupt();  
      //Timer1.stop();
      return true;
    }
    delay(5);
  }
  Serial.println(F("stto"));  
  Timer1.detachInterrupt();
  //Timer1.stop();
  return false;
}

boolean check_text_mode(int retry)//V 1.1
{
  char cache;
  String tmps="";
  tmps.reserve(128);  
  gsm_m.flush(); 
  gsm_m.print("AT+CMGF?\r");    //Send the SMS in text mode
  //delay(10);
  //Serial.println(F("check text mode"));   
  //tmps="";  
  Timer1.attachInterrupt(timer_1s);
  // Timer1.start();
  timeout_timer=0;
  while(timeout_timer<retry)
  {
    while(gsm_m.available()>0) 
    {
      cache=gsm_m.read();
      tmps=tmps+cache;
      //delayMicroseconds(200);
    }

    //Serial.print(tmps);
    if(tmps.indexOf("+CMGF: 1")!=-1)
    {
      Serial.println(F("cts")); 
      //Serial.println(tmps); 
      Timer1.detachInterrupt();
      //Timer1.stop();
      return true;
    }
    else
    {
      if(tmps.indexOf("ERROR")!=-1)
      {
        Serial.println(F("cte")); 
        Timer1.detachInterrupt();
        //Timer1.stop();
        return false;
      }
    }
    delay(5);
  }
  Serial.println(F("ctts"));   
  Timer1.detachInterrupt();
  //Timer1.stop();
  return false; 
}

boolean gsm_send(char* c_num,int msec,String c_text)//V 1.3
{
  boolean sent=false;
  int count=0;
  int retry=0;
  String tmps="";
  tmps.reserve(128); 
  while(!check_text_mode(10))
  {
    set_text_mode(10);
    Serial.println(F("not text mode"));
    delay(50);
    retry++;
    if(retry>3)
    {
      return false;
    }
  }

  gsm_m.flush();
  gsm_m.print("AT+CMGS=\"");
  for(int a=0;a<13;a++)
  {
    gsm_m.print(c_num[a]);
  }
  gsm_m.println("\"");
  delay(50);

  gsm_m.println(c_text);//the content of the message
  delay(50);

  gsm_m.write((char)26);//ctrl+z is 26
  delay(50);

  gsm_m.println();
  //gsm_m.flush();
  //delay(15000);
  //tmps="";
  Timer1.attachInterrupt(timer_1s);
  //Timer1.start();
  timeout_timer=0;
  while(timeout_timer<msec)
  {
    while(gsm_m.available()>0) 
    {
      char cache=gsm_m.read();
      tmps=tmps+cache;
    }
    //Serial.print("return sms re");
    //Serial.println(tmps);
    if(tmps.indexOf("+CMGS:")!=-1)
    {
      Serial.println(F("gss")); 
      Timer1.detachInterrupt();
      //Timer1.stop();
      return true;
    }
    else
    {
      if(tmps.indexOf("ERROR")!=-1)
      {
        Serial.println(F("gse")); 
        Timer1.detachInterrupt();
        //Timer1.stop();
        return false;
      }
    }  
    delay(5);
  }
  Serial.println(F("gsto")); 
  Timer1.detachInterrupt();
  //Timer1.stop();
  return false;  
}
////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
void EEPROMwrite(unsigned long address , unsigned char data)
{
  //digitalWrite(8,HIGH);
  delay(15);
  Wire.beginTransmission((unsigned char)(0x50 + (address>>16)));
  Wire.write((unsigned char)(address >> 8));   // MSB  
  Wire.write((unsigned char)(address)); // LSB  
  Wire.write(data); 
  Wire.endTransmission();
  delay(15);
  //digitalWrite(8,LOW);
}

unsigned char  EEPROMread(unsigned long address)
{
  unsigned char readdata = 0;
  //digitalWrite(8,HIGH);
  delay(15);
  Wire.beginTransmission((unsigned char)(0x50+(address>>16))); 
  Wire.write((unsigned char)(address >> 8));   // MSB
  Wire.write((unsigned char)(address)); // LSB
  Wire.endTransmission();
  //console.println("start reading....");   
  Wire.requestFrom(0x50,1);
  if(Wire.available())
  {
    readdata = Wire.read();
  }   
  //console.print("Read data is ");
  //console.println(readdata);  
  delay(15);
  //digitalWrite(8,LOW); 
  return readdata;

}


void make_sms(void)
{
  int reg=0;//to save reg information  add by ciou 
  int signal=0;//to save gsm sigal addby ciou
  //Serial.println(F("se"));

  int gsm_retry_count=0;//21
  gsmsuss=false;
  delay(10000);          //150918 by shrimp
  digitalWrite(GSM_SW,HIGH);//21turn on

  delay(2000);//21
  gsm_initial();//21
  while(gsm_retry_count<Gsm_retry&&!gsmsuss)
  {

    //wait
    Serial.println(F("wgsm"));
    signal = gsm_signal();
    Serial.print(F("Gs: "));
    Serial.println(signal,DEC); 
    if(gsm_available(signal))
    { 
      delay(Powt);
      reg=gsm_registration_status();
      Serial.print(F("Gr: "));
      Serial.println(reg,DEC);
      if(gsm_registration(reg)) //find reg
      {

        unsigned int sretry=0;
        int sms_space=100;
        String send_sms_content="";
        send_sms_content.reserve(100);
        boolean sent;
        while(((data_offset>0) || sms_space<100)  && (sretry<Gsm_sms_retry))//return echo and data //(return_id_count>0) || 
        {
          if((data_offset>0)&&(sms_space>80))
          {
           // load_data(data_offset-1);       
            send_sms_content.concat("@DATA,");
            //////////////////////date 
            send_sms_content.concat((int)((tamp.datavaild>>0)&0x01));
            send_sms_content.concat(',');

            send_sms_content.concat((int)tamp.gpsyear+14);
            send_sms_content.concat(',');

            if(tamp.gpsmonth<10)
            {
              send_sms_content.concat('0');
            }
            send_sms_content.concat((int)tamp.gpsmonth);
            send_sms_content.concat(',');

            if(tamp.gpsday<10)
            {
              send_sms_content.concat('0');
            }
            send_sms_content.concat((int)tamp.gpsday);
            send_sms_content.concat(',');

            //////////////////time
            send_sms_content.concat((int)((tamp.datavaild>>1)&0x01));
            send_sms_content.concat(',');

            if(tamp.gpshour<10)
            {
              send_sms_content.concat('0');
            }
            send_sms_content.concat((int)tamp.gpshour);
            send_sms_content.concat(',');

            if(tamp.gpsmin<10)
            {
              send_sms_content.concat('0');
            }
            send_sms_content.concat((int)tamp.gpsmin);
            send_sms_content.concat(',');

            if(tamp.gpssec<10)
            {
              send_sms_content.concat('0');
            } 
            send_sms_content.concat((int)tamp.gpssec);
            send_sms_content.concat(',');


            ////////////////////////location
            send_sms_content.concat((int)((tamp.datavaild>>3)&0x01)); 
            send_sms_content.concat(',');
            send_sms_content.concat(tamp.longitude);
            send_sms_content.concat(',');
            send_sms_content.concat((char)tamp.lon_dir);
            send_sms_content.concat(',');

            send_sms_content.concat(tamp.latitude);
            send_sms_content.concat(',');
            send_sms_content.concat((char)tamp.lat_dir);
            send_sms_content.concat(',');
            /////////////////////heg
            send_sms_content.concat((int)((tamp.datavaild>>2)&0x01)); 
            send_sms_content.concat(',');

            send_sms_content.concat(tamp.height);
            send_sms_content.concat(',');
            /////////////////////////te
            send_sms_content.concat(tamp.temperature);
            send_sms_content.concat(',');
            ////////////////////////voltage
            send_sms_content.concat(tamp.battery);
            send_sms_content.concat(',');

            ///////////////////pre
            send_sms_content.concat(tamp.hdop_value);
            send_sms_content.concat(',');
            /////////////////ab
            send_sms_content.concat(tamp.average_vab_count);
            send_sms_content.concat(',');
            //////////////index
            send_sms_content.concat(tamp.index);

            send_sms_content.concat(",#");

            sms_space-=80;

            clear_data();
          }
          if(((sms_space<81) && (data_offset>0)) || (data_offset==0)) 
          {
            //send_sms_content.toCharArray(SMS_buffer,send_sms_content.length());
            if(gsm_send(Number1,30,send_sms_content))
            {
              send_sms_content="";
              sms_space=100;
              if(data_offset<=eeprom_pointer)
              {
                eeprom_pointer--;
              }
              data_offset--;
              save_data_index();  
              if(data_offset==0)
              {
                gsmsuss=true;
              }
            }
            else
            {
              sretry++;
              if(sretry>=Gsm_sms_retry)
              {
                gsmsuss=true;
              }

              // delay(100);  
            }  
          } 
        }
        // Serial.println(F("gsm suss"));  
      }
      else
      {
        //Serial.print(F("fail!reg status=") );
        if(gsm_retry_count%2==1)
        {
          digitalWrite(GSM_SW,LOW);//off
          delay(1000);
          digitalWrite(GSM_SW,HIGH);//on
          //delay(5000);
        }
        //Serial.println(reg,DEC);
        gsm_retry_count++;
        // delay(100);
      }  
    }
    else
    {
      signal = 0;

      gsm_retry_count++;
      // delay(100);
      //Serial.println(F("Reading Error"));
    }

  }
  digitalWrite(GSM_SW,LOW);//off 
}
///////////////////////////////////////////////////////////////////setup

void setup(){
  //console.println(F("System start"));
  Wire.begin(); 
  pinMode(8,OUTPUT);
  digitalWrite(8,LOW);
  pinMode(led,OUTPUT);
  pinMode(GPS_SW,OUTPUT);
  pinMode(WDInput,OUTPUT);
  pinMode(GSM_SW,OUTPUT);
  pinMode(sv_sw,OUTPUT);
  digitalWrite(WDInput,LOW);
  digitalWrite(sv_sw,LOW);
  digitalWrite(led,LOW);
  digitalWrite(GPS_SW,LOW);
  digitalWrite(GSM_SW,LOW);
  //pinMode(led,OUTPUT);
  //digitalWrite(led,HIGH);
  Serial.begin(9600);
  //gsm intial
  //console.begin(9600);
  gsm_m.begin(9600);

#if FASTADC
  // set prescale to 128
  sbi(ADCSRA,ADPS2) ;
  sbi(ADCSRA,ADPS1) ;
  sbi(ADCSRA,ADPS0) ;
#endif

  Timer1.initialize(1000000);                      //set timer period = 1s
  //Timer1.attachInterrupt(timer_1s);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  MCUSR &= ~(1 << WDRF);                           // reset status flag
  WDTCSR |= (1 << WDCE) | (1 << WDE);              // enable configuration changes
  WDTCSR = (1<< WDP0) | (1 << WDP3) ;              // set the prescalar = 9
  WDTCSR |= _BV(WDIE);
  //WDTCSR |= (1 << WDIE);                           // enable interrupt mode
  //clear_eeprom();
  //print_eeprom();
  //console.println(F("initializing done\n\r\n\r"));
  delay(100);
  //EEPROM.write(0, 0);
  //EEPROM.write(1, 0);
  load_data_index();


  int batteryvol=0;
  adcsra_save = ADCSRA;
  Serial.print(F("adc="));
  Serial.println(adcsra_save,DEC); 

  batteryvol=get_voltage(); 
  if(batteryvol>gsm_hold_voltage)
  {

    int gsm_retry_count=0;
    int gsm_sms_retry_count=0;
    boolean sent=false;
    int sms_space=100;
    String send_sms_content="";
    digitalWrite(GSM_SW,HIGH);//turn on
    send_sms_content.reserve(100);
    while((gsm_retry_count<Gsm_retry)&&(!sent))
    {

      int signal=0;
      int reg=0;
      delay(Powt);
      gsm_initial();
      Serial.println(F("wgsm"));
      signal = gsm_signal();
      Serial.print(F("Gs: "));
      Serial.println(signal,DEC); 
      if(gsm_available(signal))
      { 
        reg=gsm_registration_status();
        Serial.print(F("Gr: "));
        Serial.println(reg,DEC);
        if(gsm_registration(reg))
        {
          set_text_mode(10);
          send_sms_content=""; 
          send_sms_content.concat("@Up,");
          send_sms_content.concat(batteryvol);
          send_sms_content.concat(",");
          send_sms_content.concat(signal);
          
          while((gsm_sms_retry_count<Gsm_sms_retry))
          {
            if(check_text_mode(10))//(gsm_send(Number1,30,send_sms_content))
            {
              batteryvol=get_voltage();
              get_GPS_data(batteryvol);
              make_sms();
              sent=true;
              send_sms_content="";
              Serial.println("sent"); 
              break;
            }
            else
            {
              gsm_sms_retry_count++;
              gsm_retry_count++;
            }
          }

        }
        else
        {
          //Serial.print("fail!reg status=" );
          if(gsm_retry_count%2==1)
          {
            digitalWrite(GSM_SW,LOW);//off
            delay(1000);
            digitalWrite(GSM_SW,HIGH);//on
            //delay(5000);
          }
          //Serial.println(reg,DEC);
          gsm_retry_count++;
        }  
      }
      else
      {
        signal = 0;

        gsm_retry_count++;
        //Serial.println("Reading Error");

      }

    }
    digitalWrite(GSM_SW,LOW);//off
  }
  if(batteryvol<=gsm_hold_voltage)
  {
    for(int i=0;i<3;i++)
    {
      digitalWrite(led,HIGH);
      delay(3000);
      digitalWrite(led,LOW);
      delay(1000);
    }
  }
  //Serial.flush();
  batteryvol=get_voltage();
  get_GPS_data(batteryvol);   //collect GPS data
  //turn on gps and get data
  //save data from buffer to EEPROM


  
  if((batteryvol>gsm_hold_voltage)&&(data_offset>0))
  {
    make_sms();
  }

  attachInterrupt(1, vab_count,RISING );
}

///////////////////////////////////////////////////////////////////loop

void loop()
{
  //Serial.begin(9600);
  //  batteryvol=readVcc();  mark by Linkai
  Serial.println(F("wa.."));
  WD_counter = WD_out;
  if(gps_flag)
  { 
   int batteryvol=get_voltage();  
    gps_timer = 0 ;
    gpsonce=true;
    gpssuss=false;


    //Serial.flush();
    get_GPS_data(batteryvol);   //collect GPS data
    //turn on gps and get data
    //save data from buffer to EEPROM

    gps_flag = false;  
    gpssuss=true;
    //Serial.println(F("gps suss"));
    Serial.println(F("gpe2"));


    
    if((batteryvol>gsm_hold_voltage)&&(data_offset>=data_send_count))
    {

      make_sms();
     
    }
  }


  Serial.flush();

  sleep_enable();                                  // enable the sleep mode ready for use
  wdt_reset();
  digitalWrite(7,LOW);
  digitalWrite(6,LOW);
  sleep_mode();                                    // trigger the sleep
  sleep_disable();                                 // prevent further sleep
  //power_all_enable();
}
