// This #include statement was automatically added by the Particle IDE.
#include "MQTT/MQTT.h"

#include <math.h>

#define REED0 D2 // Reed0 - Window Left
#define REED1 D4 // Reed1 - Window Right
#define PIR D7 // PIR - Motion Detection
#define LDR A0 // LDR - Light Intensity
#define DHTPIN D6 // DHT-Pin - Temperature / Humidity
#define DHTTYPE DHT11

#define boolean bool
#define BMP085_DEBUG 0
#define BMP085_I2CADDR 0x77
#define BMP085_ULTRALOWPOWER 0
#define BMP085_STANDARD      1
#define BMP085_HIGHRES       2
#define BMP085_ULTRAHIGHRES  3
#define BMP085_CAL_AC1           0xAA  // R   Calibration data (16 bits)
#define BMP085_CAL_AC2           0xAC  // R   Calibration data (16 bits)
#define BMP085_CAL_AC3           0xAE  // R   Calibration data (16 bits)
#define BMP085_CAL_AC4           0xB0  // R   Calibration data (16 bits)
#define BMP085_CAL_AC5           0xB2  // R   Calibration data (16 bits)
#define BMP085_CAL_AC6           0xB4  // R   Calibration data (16 bits)
#define BMP085_CAL_B1            0xB6  // R   Calibration data (16 bits)
#define BMP085_CAL_B2            0xB8  // R   Calibration data (16 bits)
#define BMP085_CAL_MB            0xBA  // R   Calibration data (16 bits)
#define BMP085_CAL_MC            0xBC  // R   Calibration data (16 bits)
#define BMP085_CAL_MD            0xBE  // R   Calibration data (16 bits)

#define BMP085_CONTROL           0xF4
#define BMP085_TEMPDATA          0xF6
#define BMP085_PRESSUREDATA      0xF6
#define BMP085_READTEMPCMD       0x2E
#define BMP085_READPRESSURECMD   0x34
#define MAXTIMINGS 85

#define DHT11 11
#define DHT22 22
#define DHTNAN 999999

class DHT {
private:
    uint8_t data[6];
    uint8_t _pin, _type, _count;
    bool read(void);
    unsigned long _lastreadtime;
    bool firstreading;
public:
    DHT(uint8_t pin, uint8_t type, uint8_t count=6);
    void begin(void);
    float readTemperature(void);
    float convertCtoF(float);
    float readHumidity(void);
};

DHT::DHT(uint8_t pin, uint8_t type, uint8_t count) {
    _pin = pin;
    _type = type;
    _count = count;
    firstreading = true;
}

void DHT::begin(void) {
    // set up the pins!
    pinMode(_pin, INPUT);
    digitalWrite(_pin, HIGH);
    _lastreadtime = 0;
}

float DHT::readTemperature(void) {
    float _f;
    if (read()) {
        switch (_type) {
            case DHT11:
                _f = data[2];
                return _f;
                
            case DHT22:
                _f = data[2] & 0x7F;
                _f *= 256;
                _f += data[3];
                _f /= 10;
                
                if (data[2] & 0x80)
                    _f *= -1;
                return _f;
        }
    }
    
    return DHTNAN;
}

float DHT::readHumidity(void) {
    float _f;
    if (read()) {
        switch (_type) {
            case DHT11:
                _f = data[0];
                return _f;
            case DHT22:
                _f = data[0];
                _f *= 256;
                _f += data[1];
                _f /= 10;
                return _f;
        }
    }
    return DHTNAN;
}

bool DHT::read(void) {
    uint8_t laststate = HIGH;
    uint8_t counter = 0;
    uint8_t j = 0, i;
    unsigned long currenttime;
    
    // pull the pin high and wait 250 milliseconds
    digitalWrite(_pin, HIGH);
    delay(250);
    
    currenttime = millis();
    if (currenttime < _lastreadtime) {
        // ie there was a rollover
        _lastreadtime = 0;
    }
    
    if (!firstreading && ((currenttime - _lastreadtime) < 2000)) {
        //delay(2000 - (currenttime - _lastreadtime));
        return true; // return last correct measurement
    }
    
    firstreading = false;
    _lastreadtime = millis();
    
    data[0] = data[1] = data[2] = data[3] = data[4] = 0;
    
    // now pull it low for ~20 milliseconds
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
    delay(20);
    noInterrupts();
    digitalWrite(_pin, HIGH);
    delayMicroseconds(40);
    pinMode(_pin, INPUT);
    
    // read in timings
    for ( i=0; i< MAXTIMINGS; i++) {
        counter = 0;
        
        while (digitalRead(_pin) == laststate) {
            counter++;
            delayMicroseconds(1);
            if (counter == 255)
                break;
        }
        
        laststate = digitalRead(_pin);
        
        if (counter == 255)
            break;
        
        // ignore first 3 transitions
        if ((i >= 4) && (i%2 == 0)) {
            // shove each bit into the storage bytes
            data[j/8] <<= 1;
            if (counter > _count)
                data[j/8] |= 1;
            j++;
        }
    }
    interrupts();
    
    // check we read 40 bits and that the checksum matches
    if ((j >= 40) &&  (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)))
        return true;
    return false;
}

class Adafruit_BMP085 {
public:
    Adafruit_BMP085();
    boolean begin(uint8_t mode = BMP085_ULTRAHIGHRES);  // by default go highres
    float readTemperature(void);
    int32_t readPressure(void);
    float readAltitude(float sealevelPressure = 101325); // std atmosphere
    uint16_t readRawTemperature(void);
    uint32_t readRawPressure(void);
    
private:
    uint8_t read8(uint8_t addr);
    uint16_t read16(uint8_t addr);
    void write8(uint8_t addr, uint8_t data);
    
    uint8_t oversampling;
    
    int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
    uint16_t ac4, ac5, ac6;
};


Adafruit_BMP085::Adafruit_BMP085() {
}

boolean Adafruit_BMP085::begin(uint8_t mode) {
    if (mode > BMP085_ULTRAHIGHRES)
        mode = BMP085_ULTRAHIGHRES;
    oversampling = mode;
    
    Wire.begin();
    
    if (read8(0xD0) != 0x55) return false;
    
    /* read calibration data */
    ac1 = read16(BMP085_CAL_AC1);
    ac2 = read16(BMP085_CAL_AC2);
    ac3 = read16(BMP085_CAL_AC3);
    ac4 = read16(BMP085_CAL_AC4);
    ac5 = read16(BMP085_CAL_AC5);
    ac6 = read16(BMP085_CAL_AC6);
    
    b1 = read16(BMP085_CAL_B1);
    b2 = read16(BMP085_CAL_B2);
    
    mb = read16(BMP085_CAL_MB);
    mc = read16(BMP085_CAL_MC);
    md = read16(BMP085_CAL_MD);
    return true;
}

uint16_t Adafruit_BMP085::readRawTemperature(void) {
    write8(BMP085_CONTROL, BMP085_READTEMPCMD);
    delay(5);
    return read16(BMP085_TEMPDATA);
}

uint32_t Adafruit_BMP085::readRawPressure(void) {
    uint32_t raw;
    
    write8(BMP085_CONTROL, BMP085_READPRESSURECMD + (oversampling << 6));
    
    if (oversampling == BMP085_ULTRALOWPOWER)
        delay(5);
    else if (oversampling == BMP085_STANDARD)
        delay(8);
    else if (oversampling == BMP085_HIGHRES)
        delay(14);
    else
        delay(26);
    
    raw = read16(BMP085_PRESSUREDATA);
    
    raw <<= 8;
    raw |= read8(BMP085_PRESSUREDATA+2);
    raw >>= (8 - oversampling);
    
    return raw;
}

uint16_t Adafruit_BMP085::read16(uint8_t a) {
    uint16_t ret;
    
    Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
    Wire.write(a); // sends register address to read from
    Wire.endTransmission(); // end transmission
    
    Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
    Wire.requestFrom(BMP085_I2CADDR, 2);// send data n-bytes read
    ret = Wire.read(); // receive DATA
    ret <<= 8;
    ret |= Wire.read(); // receive DATA
    Wire.endTransmission(); // end transmission
    
    return ret;
}

void Adafruit_BMP085::write8(uint8_t a, uint8_t d) {
    Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
    Wire.write(a); // sends register address to read from
    Wire.write(d);  // write data
    Wire.endTransmission(); // end transmission
}

uint8_t Adafruit_BMP085::read8(uint8_t a) {
    uint8_t ret;
    
    Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
    Wire.write(a); // sends register address to read from
    Wire.endTransmission(); // end transmission
    
    Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
    Wire.requestFrom(BMP085_I2CADDR, 1);// send data n-bytes read
    ret = Wire.read(); // receive DATA
    Wire.endTransmission(); // end transmission
    
    return ret;
}

float Adafruit_BMP085::readTemperature(void) {
    int32_t UT, X1, X2, B5;     // following ds convention
    float temp;
    
    UT = readRawTemperature();
    
    // step 1
    X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) / pow(2,15);
    X2 = ((int32_t)mc * pow(2,11)) / (X1+(int32_t)md);
    B5 = X1 + X2;
    temp = (B5+8)/pow(2,4);
    temp /= 10;
    
    return temp;
}

float Adafruit_BMP085::readAltitude(float sealevelPressure) {
    float altitude;
    float pressure = readPressure();
    altitude = 44330 * (1.0 - pow(pressure /sealevelPressure,0.1903));
    return altitude;
}

int32_t Adafruit_BMP085::readPressure(void) {
    int32_t UT, UP, B3, B5, B6, X1, X2, X3, p;
    uint32_t B4, B7;
    
    UT = readRawTemperature();
    UP = readRawPressure();
    
    // do temperature calculations
    X1=(UT-(int32_t)(ac6))*((int32_t)(ac5))/pow(2,15);
    X2=((int32_t)mc*pow(2,11))/(X1+(int32_t)md);
    B5=X1 + X2;
    
    // do pressure calcs
    B6 = B5 - 4000;
    X1 = ((int32_t)b2 * ( (B6 * B6)>>12 )) >> 11;
    X2 = ((int32_t)ac2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = ((((int32_t)ac1*4 + X3) << oversampling) + 2) / 4;
    
    X1 = ((int32_t)ac3 * B6) >> 13;
    X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
    B7 = ((uint32_t)UP - B3) * (uint32_t)( 50000UL >> oversampling );
    
    if (B7 < 0x80000000) {
        p = (B7 * 2) / B4;
    } else {
        p = (B7 / B4) * 2;
    }
    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;
    
    p = p + ((X1 + X2 + (int32_t)3791)>>4);
    return p;
}

float oversampleRead(uint16_t input_pin,uint8_t times) {
	float average=0.0;
	for (uint8_t i=0;i<times;i++) {
        average+=analogRead(input_pin);
	}
	average=average/times;
	return average;
}

DHT dht(DHTPIN, DHTTYPE);
Adafruit_BMP085 bmp;

bool dhtfail=0; // valid DHT readings ?
float dhthumidity; // DHT humidity
float dhttemperature; // DHT temperature
char humidity[5]; // humidity
char temperature[5]; // temperature
char reed0[2]; // left window
char reed1[2]; // right window
char motion[2]; // PIR motion
char light[5]; // light intensity
char altitude[5]; // altitude
char pressure[5]; // pressure
char bmptemp[5]; // Spark Core inner case temperature

const unsigned int loopdelay=3000; // only query sensors every 3 seconds
unsigned long looptime;
bool sending = false;	// Failsafe to make sure we don't try to post while already posting (this is probably entirely unneeded, but I'm paranoid)
unsigned int lowcount=0;

void mqtt_cb(char* topic, byte* payload, unsigned int length);
byte mqtt_ip[] = { 10, 1, 1, 212 };
MQTT mqtt(mqtt_ip, 1883, mqtt_cb);
void mqtt_cb(char* topic, byte* payload, unsigned int length) {
}

void setup() {
 pinMode(REED0, INPUT_PULLUP);
 pinMode(REED1, INPUT_PULLUP);
 pinMode(PIR, INPUT);
 pinMode(LDR, INPUT);
 mqtt.connect("sparkcore3");
 mqtt.publish("sparkcore/3/status", "setup");
 dht.begin();
 bmp.begin();
 looptime=millis();
}

void loop() {
   if ((millis() > looptime + loopdelay) && sending==false) {
     if (mqtt.isConnected()) { mqtt.loop(); } else { mqtt.connect("sparkcore3"); };
     sending = true;
     mqtt.publish("sparkcore/3/status", "start");
     
     // Check left window reed
     if (digitalRead(REED0)) {
        sprintf(reed0,"0");
     }  else {
        sprintf(reed0,"1");
     }
     mqtt.publish("sparkcore/3/reed0", reed0);
     
     // Check right window reed
     if (digitalRead(REED1)) {
        sprintf(reed1,"0");
     }  else {
        sprintf(reed1,"1");
     }
     mqtt.publish("sparkcore/3/reed1", reed1);
     
     // Check PIR motion
     if (digitalRead(PIR)) {
        sprintf(motion,"1");
     }  else {
        sprintf(motion,"0");
     }
     mqtt.publish("sparkcore/3/motion", motion);
     
     lowcount++;
     if (lowcount >= 10) { // report those less often     
       // Check light intensity
       sprintf(light,"%0.1f",(oversampleRead(LDR,8)/40.96));
       mqtt.publish("sparkcore/3/light", light);
     
       // Check pressure
       sprintf(pressure,"%.0f",(float)(bmp.readPressure()/100));
       mqtt.publish("sparkcore/3/pressure", pressure);
     
       // Check altitude
       sprintf(altitude,"%.0f",bmp.readAltitude());
       mqtt.publish("sparkcore/3/altitude", altitude);
     
       // Check spark core inner case temperature
       sprintf(bmptemp,"%.1f",bmp.readTemperature());
       mqtt.publish("sparkcore/3/casetemp", bmptemp);
     
       // Check Humidity & Temperature
       dhtfail = 0;
       dhthumidity = dht.readHumidity();
       dhttemperature = dht.readTemperature();
       if (dhttemperature==DHTNAN || dhthumidity==DHTNAN) {
           dhtfail = 1;
       } else {
           dhtfail = 0;
           sprintf(humidity,"%.1f",dhthumidity);   
           mqtt.publish("sparkcore/3/humidity", humidity);
           sprintf(temperature,"%.1f",dhttemperature);   
           mqtt.publish("sparkcore/3/temperature", temperature);
       }
       lowcount=0;
     }
     mqtt.publish("sparkcore/3/status", "end");
     sending = false;
     looptime=millis(); 
   }
}
