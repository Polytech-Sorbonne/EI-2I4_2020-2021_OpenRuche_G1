#ifndef __PORTSTM_HH__
#define __PORTSTM_HH__

// Liaison sigfox
Serial sigfox(D1, D0); // tx, rx
// Bus OneWire pour DS1820
OneWire     oneWire(D2);
// Sondes de température étanches
DS1820*     ds1820[SENSORS_NR]; 
// Pin pour PM
DigitalOut done(D5);
// Capteur de poids
HX711 Balance(D12,D11);
// Led allumage
DigitalOut myLed(LED1) ;
//DHT 22 extérieur
DHT dhtE(D3, DHT22);

#endif