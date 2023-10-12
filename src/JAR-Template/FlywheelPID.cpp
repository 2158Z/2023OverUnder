#include "vex.h"

float Ferror;
float Fpreverror;
float Fderivative;
float Fkp  = 0.01;
float Fkd = 0.0;
float SPEED;
int DesiredSpeed;
float OutSpeed;
float TBH;
float Gain = 0.05;
float TBHSpeed;
bool Autoo;


/*
int FlySpeedCTRL()
{
    while (1)
    {
        SPEED = ((Sp.velocity(velocityUnits::rpm)*2)/3);

            Ferror = DesiredSpeed - SPEED;
            Fderivative = Fpreverror - Ferror;
        
        OutSpeed = (3.564 * DesiredSpeed) + ((Ferror * Fkp) + (Fderivative * Fkd));

        Flywheel.spin(directionType::fwd,OutSpeed,voltageUnits::mV);

        Fpreverror = Ferror;

        printf(" %f\n", (Sp.velocity(velocityUnits::rpm)));

     wait (20,msec);
    }
return 0;
}
*/
int FlySpeedCTRL()
{
    while (Autoo)
    {
    
        SPEED = ((Sp.velocity(velocityUnits::rpm)));

        if (SPEED > (DesiredSpeed - 200))
        {

            Ferror = DesiredSpeed - SPEED;

            TBHSpeed += Ferror * Gain;

            if (signbit(Ferror)!= signbit(Fpreverror))
            {

                TBHSpeed = .5*(TBHSpeed + TBH);

                TBH = TBHSpeed;
  
            }
        }

        OutSpeed = (3.12 * DesiredSpeed) + TBHSpeed;

        Flywheel.spin(directionType::fwd,OutSpeed,voltageUnits::mV);
        
        Fpreverror = Ferror;


     wait (20,msec);
    }
return 0;
}


