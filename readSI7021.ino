#include <LowPower.h>

#include <Wire.h>
#include <SI7021.h>


SI7021 sensor;
const int m_time = 5;      //Meassuretime in Seconds
int wind_ct = 0;
float wind = 0.0;
unsigned long time = 0;
void setup() {
    Serial.begin(9600);
    sensor.begin();
    time = millis();
}
 void countWind() {
       wind_ct ++;
    }
    void mesure() {
    wind_ct = 0;
    time = millis();
    attachInterrupt(1, countWind, RISING);
    delay(1000 * m_time);
    detachInterrupt(1);
    wind = (float)wind_ct / (float)m_time * 2.4;
    }


void loop() {
  //Mode sommeil
    LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, 
                SPI_OFF, USART0_OFF, TWI_OFF);

                
    mesure();

    Serial.print("vitesse du vent: ");
    Serial.print(wind);       //Speed in Km/h
    Serial.print(" km/h - ");
    Serial.print(wind / 3.6); //Speed in m/s
    Serial.println(" m/s");

    delay(500);

    
    // temperature is an integer in hundredths
    int temperature = sensor.getCelsiusHundredths();
    temperature = temperature / 100;
    Serial.print("Temperature :");  
    Serial.print(temperature);
    Serial.println("°Celcius");
    
    delay(500);
    
    // humidity is an integer representing percent
    int humidity = sensor.getHumidityPercent();
    Serial.print("Humidity :");  
    Serial.print(humidity);
    Serial.println("%");
   
       delay(500);
}
