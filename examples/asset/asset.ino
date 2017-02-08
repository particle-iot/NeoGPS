#include <Arduino.h>
#include <atv2.h>

SYSTEM_THREAD(ENABLED);
//SYSTEM_MODE(MANUAL);

// Creating an AssetTracker named 't' for us to reference
AssetTracker t = AssetTracker();

//--------------------------

void setup()
{
    // Sets up all the necessary AssetTracker bits
    t.begin();  
    t.gpsOn();

    //Start a debug channel on Serial
    Serial.begin(9600);

    Serial.println("Starting asset tracker demo");


}

//--------------------------

void loop()
{
  t.updateGPS();

  Particle.publish("LA", String(t.readLat()),60,PRIVATE);
  Particle.publish("LO", String(t.readLon()),60,PRIVATE);
  Particle.publish("T", String(t.getGpsTimestamp()),60,PRIVATE);
  Particle.publish("F", String(t.gpsFix()),60,PRIVATE);

  //String pubAccel = String::format("%d,%d,%d",t.readX(),t.readY(),t.readZ());
  //Serial.println("A", pubAccel,60,PRIVATE);

   delay(5000);

}
