#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Arduino.h>

// Forward declarations of functions (so they can be called before defined)
float readBatteryVoltage(int pin, float dividerRatio);
bool updateBatteryStatus(bool forceUpdate = false);
void printBatteryStatus();

int motorBatteryPin = A0;
int logicBatteryPin = A1;

// Last check time for non-blocking updates
unsigned long lastBatteryCheckTime = 0;
unsigned long batteryCheckInterval = 5000; // 5 seconds between checks

// Battery status variables
float motorBatteryVoltage = 0.0;
float logicBatteryVoltage = 0.0;
bool motorBatteryLow = false;
bool logicBatteryLow = false;
bool motorLowNotified = false;
bool logicLowNotified = false;

// Battery monitoring settings
float internalRefVoltage = 2.56; // Internal reference voltage for ATmega32U4
float motorDividerRatio = 5.0;   // Calibrated based on actual measurements: Conversion Logic: (100k + 20k) // 20k = 6.0
float logicDividerRatio = 2.0;   // Conversion Logic: (10k + 10k) // 10k = 2.0

float motorLowThreshold = 10.5; // 90% of 12V
float logicLowThreshold = 3.0;  // 50% of 6V
bool batteryDebug = false;

// Initialize the battery monitor
void batteryMonitorInit(int mPin, int lPin, float mRatio, float lRatio, bool debug = false)
{
    motorBatteryPin = mPin;
    logicBatteryPin = lPin;
    motorDividerRatio = mRatio;
    logicDividerRatio = lRatio;

    // Only override divider ratio if a non-zero value is provided
    if (mRatio > 0)
    {
        motorDividerRatio = mRatio;
    }

    batteryDebug = debug;

    // Switch to internal reference for more stable readings
    analogReference(INTERNAL);
    // Delay to allow reference to stabilize
    delay(10);
    // Discard first few readings after changing reference
    for (int i = 0; i < 5; i++)
    {
        analogRead(motorBatteryPin);
        delay(5);
    }

    // Take initial readings
    updateBatteryStatus(true);

    if (batteryDebug)
    {
        Serial.println("Battery monitor initialized");
        printBatteryStatus();
    }
}

// Read a battery voltage through a voltage divider
float readBatteryVoltage(int pin, float dividerRatio)
{
    // Take multiple readings for stability
    long total = 0;
    for (int i = 0; i < 5; i++)
    {
        total += analogRead(pin);
        delay(2); // Short delay between readings
    }
    int rawValue = total / 5; // Average of 5 readings

    // Convert to voltage using internal reference
    float voltage = (rawValue / 1023.0) * internalRefVoltage * dividerRatio;

    if (batteryDebug)
    {
        Serial.print("Raw analog (pin ");
        Serial.print(pin);
        Serial.print("): ");
        Serial.print(rawValue);
        Serial.print(" -> ");
        Serial.print(voltage);
        Serial.println("V");
    }

    return voltage;
}

// Update battery status - returns true if readings were updated
bool updateBatteryStatus(bool forceUpdate)
{
    unsigned long currentTime = millis();

    if (forceUpdate || (currentTime - lastBatteryCheckTime >= batteryCheckInterval))
    {
        // Read both batteries
        motorBatteryVoltage = readBatteryVoltage(motorBatteryPin, motorDividerRatio);
        logicBatteryVoltage = readBatteryVoltage(logicBatteryPin, logicDividerRatio);

        // Update status flags
        bool prevMotorLow = motorBatteryLow;
        bool prevLogicLow = logicBatteryLow;

        motorBatteryLow = (motorBatteryVoltage < motorLowThreshold);
        logicBatteryLow = (logicBatteryVoltage < logicLowThreshold);

        // Reset notification flags when state changes to low
        if (motorBatteryLow && !prevMotorLow)
        {
            motorLowNotified = false;
        }
        if (logicBatteryLow && !prevLogicLow)
        {
            logicLowNotified = false;
        }

        lastBatteryCheckTime = currentTime;
        return true;
    }

    return false;
}

// Print the current battery status
void printBatteryStatus()
{
    Serial.println("-------- Battery Status --------");

    // Motor battery
    Serial.print("Motor (12V): ");
    Serial.print(motorBatteryVoltage);
    Serial.print("V (");
    Serial.print((motorBatteryVoltage / 12.0) * 100.0);
    Serial.print("%) - ");
    Serial.print(motorBatteryLow ? "LOW" : "OK");
    Serial.print(" Threshold: ");
    Serial.print(motorLowThreshold);
    Serial.println("V");

    // Logic battery
    Serial.print("Arduino (6V): ");
    Serial.print(logicBatteryVoltage);
    Serial.print("V (");
    Serial.print((logicBatteryVoltage / 6.0) * 100.0);
    Serial.print("%) - ");
    Serial.print(logicBatteryLow ? "LOW" : "OK");
    Serial.print(" Threshold: ");
    Serial.print(logicLowThreshold);
    Serial.println("V");

    Serial.println("--------------------------------");
}

// Check if motor battery is low (one-time notification)
bool checkMotorBatteryLowOnce()
{
    if (motorBatteryLow && !motorLowNotified)
    {
        motorLowNotified = true;
        return true;
    }
    return false;
}

// Check if logic battery is low (one-time notification)
bool checkLogicBatteryLowOnce()
{
    if (logicBatteryLow && !logicLowNotified)
    {
        logicLowNotified = true;
        return true;
    }
    return false;
}

// Set debugging on/off
void setBatteryDebug(bool debug)
{
    batteryDebug = debug;
}

// Set battery check interval (in milliseconds)
void setBatteryCheckInterval(unsigned long interval)
{
    batteryCheckInterval = interval;
}

// Set battery thresholds
void setBatteryThresholds(float motorThreshold, float logicThreshold)
{
    motorLowThreshold = motorThreshold;
    logicLowThreshold = logicThreshold; // Using the now uncommented variable
}

#endif // BATTERY_MONITOR_H