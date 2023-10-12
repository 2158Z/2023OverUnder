#include "vex.h"
int MTemp ()
{
 while (true)
 {
  Brain.Screen.setCursor(1, 30); 
   Brain.Screen.print("FLM:"); 
   Brain.Screen.setCursor(2, 30); 
   Brain.Screen.print("FRM:"); 
   Brain.Screen.setCursor(3, 30); 
   Brain.Screen.print("BLM:"); 
   Brain.Screen.setCursor(4, 30); 
   Brain.Screen.print("BRM:"); 
   Brain.Screen.setCursor(5, 30);
   Brain.Screen.print("intake:"); 
   Brain.Screen.setCursor(6, 30); 
   Brain.Screen.print("Roller:"); 
   Brain.Screen.setCursor(7, 30); 
   Brain.Screen.print("Fly1:"); 
   Brain.Screen.setCursor(8, 30); 
   Brain.Screen.print("Fly2:");
   Brain.Screen.setCursor(1, 40); 
   Brain.Screen.print(FLM.temperature(temperatureUnits::fahrenheit)); 
   Brain.Screen.setCursor(2, 40); 
   Brain.Screen.print(FRM.temperature(temperatureUnits::fahrenheit)); 
   Brain.Screen.setCursor(3, 40); 
   Brain.Screen.print(BLM.temperature(temperatureUnits::fahrenheit)); 
   Brain.Screen.setCursor(4, 40); 
   Brain.Screen.print(BRM.temperature(temperatureUnits::fahrenheit)); 
   Brain.Screen.setCursor(5, 40);
   Brain.Screen.print(Intake.temperature(temperatureUnits::fahrenheit)); 
   Brain.Screen.setCursor(6, 40); 
   Brain.Screen.print(Rollshoot.temperature(temperatureUnits::fahrenheit)); 
   Brain.Screen.setCursor(7, 40); 
   Brain.Screen.print(flywheel1.temperature(temperatureUnits::fahrenheit)); 
   Brain.Screen.setCursor(8, 40); 
   Brain.Screen.print(flywheel2.temperature(temperatureUnits::fahrenheit));
   task::sleep(5);

 }

  return 0;
}