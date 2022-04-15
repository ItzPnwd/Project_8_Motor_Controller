// Last edit 3/23/2022
#include "mbed.h"
#include <iostream>

AnalogIn TemperatureSensor(PTB0);
AnalogIn TorqueSensor(PTB2);
InterruptIn START_BUTTON(PTD4);
InterruptIn STOP_BUTTON(PTA12);
DigitalOut RED_LED(LED1);
DigitalOut GREEN_LED(LED2);
DigitalOut BLUE_LED(LED3);
PwmOut OUTPUT(PTC2);

#define Vsupply 3.3f //microcontroller voltage supply 3.3V
#define ThermistorBiasResistor 10000.0f //Bias resistor (lower leg of voltage divider) for thermistor
#define MotorSeriesResistance 10.0f //resistance of torque (current) sensing resistor in series with the Motor

bool Running = false;

//variables for temperature sensor
float TemperatureSensorDigiValue; //the A/D converter value read by the controller input pin
float TemperatureSensorVoltValue; //the voltage on the controller input pin (across the 10k resistor) from the temperature sensor voltage divider
float ThermistorResistance; //computed from the voltage drop value across the thermistor
float ThermistorTemperature; //approximate ambient temperature measured by thermistor
float DutyCycle;                //Duty cycle of the motor to control its speed

//variables for torque sensor
float MotorCurrentDigiValue; //the A/D converter value ready by the controller input pin
float MotorCurrentVoltValue; //the voltage on the controller input pin (across the 10 ohm resistor) from the motor torque sensor
float MotorCurrent; //computed from the voltage value

// Variables to hold control reference values.
float TemperatureLimit = 20.9; //enter a temperature in Celsius here for temperature deactivation; NOTE: room temperature is 25C
float MotorCurrentLimit = 0.1; //enter a reference current in amperes for motor torque deactivation

// This function will be attached to the start button interrupt.
void StartPressed(void)
{
    cout << "Start!" << endl;
    OUTPUT = 1;
    Running = true;
}

// This function will be attached to the stop button interrupt.
void StopPressed(void)
{
    // STUDENT: EDIT HERE
    cout << "Stop!" << endl;
    OUTPUT = 0;
    Running = false;
}

// This function converts the voltage value from the thermistor input to an approximate temperature
// in Celsius based on a linear approximation of the thermistor.
float getThermistorTemperature(void)
{
    TemperatureSensorDigiValue = TemperatureSensor.read();                                                          // 1. Read the TemperatureSensor A/D value and store it in TemperatureSensorDigiValue
    TemperatureSensorVoltValue = Vsupply*TemperatureSensorDigiValue;                                                // 2. Calculate TemperatureSensorVoltValue from TemperatureSensorDigiValue and Vsupply
    ThermistorResistance = (Vsupply*ThermistorBiasResistor/TemperatureSensorVoltValue) - ThermistorBiasResistor;    // 3. Calculate ThermistorResistance using the voltage divider equation

    ThermistorTemperature = ((ThermistorResistance - 10000.0)/(-320.0)) + 25.0; //temperature of the thermistor computed by a linear approximation of the device response

    //cout <<"Digi Volt Value: " <<TemperatureSensorDigiValue << "\r\n";
    //cout <<"Volt Value: " <<TemperatureSensorVoltValue << "\r\n";
    //wait(5);
    //cout <<"Temp Resistance: " <<ThermistorResistance << "\r\n";

    return ThermistorTemperature;
}

//This function will check for a temperature triggered the activation of the motor with a motor speed corresponding to the ambient temperature
//70 deg F, 80 deg F, 90 deg F;21 deg C, 26.667 deg C, 32 deg C
//((temp-21)/11 *.5) +.5
void CheckTemperatureSensor(void)
{
    // Use the getThermistorTemperature() function defined above to obtain a temperature value to use for comparison and decision making with your TemperatureLimit
    float CurrentTemperature = getThermistorTemperature();
    if(CurrentTemperature <= TemperatureLimit) {
        cout << "Temperature Stop!" << endl;
        OUTPUT = 0; //Motor OFF
        RED_LED = 0; // ON
    } else {
        DutyCycle = ((CurrentTemperature-21)/11); 
        cout << "Changing duty cycle to: " << DutyCycle<< endl;
        OUTPUT.period(0.01f);        //Experimental. 4 second Period
        OUTPUT.write(DutyCycle);    //duty Cycle %. Ranges from 0-1, relative to period
        //OUTPUT = dCycle;          //shorthand for OUTPUT.write
        //OUTPUT.pulsewidth(2);     //alternative to OUTPUT.write, set duty cycle to time in seconds
        RED_LED = 1; // OFF
    }
}

//This function will determine the motor current in amperes
float getMotorCurrent(void)
{
    MotorCurrentDigiValue = TorqueSensor.read();                // 1. Read the TorqueSensor value and store it in MotorCurrentDigiValue
    MotorCurrentVoltValue = Vsupply*MotorCurrentDigiValue;      // 2. Calculate MotorCurrentVoltValue from MotorCurrentDigiValue and Vsupply
    MotorCurrent= MotorCurrentVoltValue/MotorSeriesResistance;  // 3. Calculate MotorCurrent using Ohm's law from MotorCurrentVoltValue and MotorSeriesResistance

    return MotorCurrent;
}

// Standard entry point in C++.
int main(void)
{
    // Attach the functions to the hardware interrupt pins.
    START_BUTTON.rise(&StartPressed);
    STOP_BUTTON.rise(&StopPressed);
    // Initialize LED outputs to OFF (LED logic is inverted)
    RED_LED = 1;
    GREEN_LED = 1;
    BLUE_LED = 1;

    // Blink the blue LED once to indicate the code is running.
    BLUE_LED = !BLUE_LED;
    wait(1.0);
    BLUE_LED = !BLUE_LED;

JumpPos:
    while(true) 
    {
        // Check the analog inputs.
        CheckTemperatureSensor();

        // Print Analog Values to screen
        cout << "\rCurrent Temperature Value: " << getThermistorTemperature() << "\r\n";

        wait(1.0); // Wait 1 second before repeating the loop.
        if(Running == false)
        {
            while(true)
            {
                wait(1.5);  //Necessary to be able to register the start buttion being presseed
                OUTPUT.write(0);
                if(Running == true)
                {
                    goto JumpPos;
                }
            }

        }    
    }
}

// End of HardwareInterruptSeedCode
