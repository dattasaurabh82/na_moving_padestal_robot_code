#ifndef ROBOT_TONES_H
#define ROBOT_TONES_H

#include <TimerFreeTone.h> // TimerFreeTone library for tones instead of built in tone() func as this doesn't interfere with timer (necessaruyy for IR remote lin)

// Define a struct for musical notes
struct Note
{
    const char *name; // Note name
    int frequency;    // Frequency in Hz
};

// Create an expanded lookup table of notes
const Note notes[] = {
    // Octave 3 (lower notes)
    {"C3", 131},
    {"C#3", 139},
    {"D3", 147},
    {"D#3", 156},
    {"E3", 165},
    {"F3", 175},
    {"F#3", 185},
    {"G3", 196},
    {"G#3", 208},
    {"A3", 220},
    {"A#3", 233},
    {"B3", 247},

    // Octave 4 (middle notes)
    {"C4", 262},
    {"C#4", 277},
    {"D4", 294},
    {"D#4", 311},
    {"E4", 330},
    {"F4", 349},
    {"F#4", 370},
    {"G4", 392},
    {"G#4", 415},
    {"A4", 440},
    {"A#4", 466},
    {"B4", 494},

    // Octave 5 (higher notes)
    {"C5", 523},
    {"C#5", 554},
    {"D5", 587},
    {"D#5", 622},
    {"E5", 659},
    {"F5", 698},
    {"F#5", 740},
    {"G5", 784},
    {"G#5", 831},
    {"A5", 880},
    {"A#5", 932},
    {"B5", 988},

    // Special value for silence
    {"REST", 0}};

// Number of notes in the lookup table
const int noteCount = sizeof(notes) / sizeof(notes[0]);

// Define indices for easier access to notes (avoiding Arduino pin name conflicts)
const int NOTE_C3 = 0, NOTE_CS3 = 1, NOTE_D3 = 2, NOTE_DS3 = 3, NOTE_E3 = 4, NOTE_F3 = 5;
const int NOTE_FS3 = 6, NOTE_G3 = 7, NOTE_GS3 = 8, NOTE_A3 = 9, NOTE_AS3 = 10, NOTE_B3 = 11;
const int NOTE_C4 = 12, NOTE_CS4 = 13, NOTE_D4 = 14, NOTE_DS4 = 15, NOTE_E4 = 16, NOTE_F4 = 17;
const int NOTE_FS4 = 18, NOTE_G4 = 19, NOTE_GS4 = 20, NOTE_A4 = 21, NOTE_AS4 = 22, NOTE_B4 = 23;
const int NOTE_C5 = 24, NOTE_CS5 = 25, NOTE_D5 = 26, NOTE_DS5 = 27, NOTE_E5 = 28, NOTE_F5 = 29;
const int NOTE_FS5 = 30, NOTE_G5 = 31, NOTE_GS5 = 32, NOTE_A5 = 33, NOTE_AS5 = 34, NOTE_B5 = 35;
const int NOTE_REST = 36;

// Define a struct for a musical sequence
struct MusicNote
{
    int noteIndex;  // Index in the notes[] array
    int duration;   // Duration in milliseconds
    int pauseAfter; // Pause after note in milliseconds
};

// Melody 1: Low Battery Warning (descending, urgent pattern)
const MusicNote lowBatteryMelody[] = {
    {NOTE_G4, 150, 50}, // G4
    {NOTE_E4, 150, 50}, // E4
    {NOTE_C4, 150, 50}, // C4
    {NOTE_G3, 450, 150} // G3 (longer)
};
const int lowBatteryMelodyLength = sizeof(lowBatteryMelody) / sizeof(lowBatteryMelody[0]);

// Melody 2: Robot Starting to Move (ascending, positive)
const MusicNote startMovingMelody[] = {
    {NOTE_C4, 100, 30}, // C4
    {NOTE_E4, 100, 30}, // E4
    {NOTE_G4, 100, 30}, // G4
    {NOTE_C5, 250, 50}  // C5 (higher, longer)
};
const int startMovingMelodyLength = sizeof(startMovingMelody) / sizeof(startMovingMelody[0]);

// Melody 3: Robot Stopped Moving (descending, conclusive)
const MusicNote stopMovingMelody[] = {
    {NOTE_C5, 150, 30}, // C5
    {NOTE_G4, 150, 30}, // G4
    {NOTE_E4, 150, 30}, // E4
    {NOTE_C4, 300, 50}  // C4 (longer)
};
const int stopMovingMelodyLength = sizeof(stopMovingMelody) / sizeof(stopMovingMelody[0]);

// Melody 4a: Robot Turning Left (distinctive pattern with descending notes)
const MusicNote turnLeftMelody[] = {
    {NOTE_E4, 80, 20}, // E4
    {NOTE_D4, 80, 20}, // D4
    {NOTE_C4, 150, 30} // C4 (longer)
};
const int turnLeftMelodyLength = sizeof(turnLeftMelody) / sizeof(turnLeftMelody[0]);

// Melody 4b: Robot Turning Right (distinctive pattern with ascending notes)
const MusicNote turnRightMelody[] = {
    {NOTE_C4, 80, 20}, // C4
    {NOTE_D4, 80, 20}, // D4
    {NOTE_E4, 150, 30} // E4 (longer)
};
const int turnRightMelodyLength = sizeof(turnRightMelody) / sizeof(turnRightMelody[0]);

// Melody: Robot Moving Backward (descending warning pattern)
const MusicNote backwardMelody[] = {
    {NOTE_G4, 100, 50},  // G4
    {NOTE_D4, 100, 100}, // D4 (lower)
    {NOTE_G4, 100, 50},  // G4 (repeat)
    {NOTE_D4, 100, 100}  // D4 (lower, repeat)
};
const int backwardMelodyLength = sizeof(backwardMelody) / sizeof(backwardMelody[0]);

// Function to play a melody
void playMelody(int buzzerPin, const MusicNote *melody, int melodyLength)
{
    for (int i = 0; i < melodyLength; i++)
    {
        if (notes[melody[i].noteIndex].frequency > 0)
        {
            // Play a tone
            // tone(buzzerPin, notes[melody[i].noteIndex].frequency, melody[i].duration);
            TimerFreeTone(buzzerPin, notes[melody[i].noteIndex].frequency, melody[i].duration);
        }
        else
        {
            // This is a rest (silence)
            // noTone(buzzerPin);
        }
        delay(melody[i].duration + melody[i].pauseAfter);
    }
    noTone(buzzerPin); // Ensure no tone is playing at the end
}

// Utility function to play a single note from the lookup table
void playNote(int buzzerPin, int noteIndex, int duration, int pauseAfter)
{
    if (notes[noteIndex].frequency > 0)
    {
        // tone(buzzerPin, notes[noteIndex].frequency, duration);
        TimerFreeTone(buzzerPin, notes[noteIndex].frequency, duration);
    }
    else
    {
        // noTone(buzzerPin);
    }
    delay(duration + pauseAfter);
}

// A custom startup sequence
void playStartupSequence(int buzzerPin)
{
    for (int i = 0; i < 3; i++)
    {
        playNote(buzzerPin, NOTE_C4 + (i * 2), 100, 50);
    }
}

// Convenience functions for each melody
void playLowBatteryTone(int buzzerPin)
{
    playMelody(buzzerPin, lowBatteryMelody, lowBatteryMelodyLength);
}

void playStartMovingTone(int buzzerPin)
{
    playMelody(buzzerPin, startMovingMelody, startMovingMelodyLength);
}

void playStopMovingTone(int buzzerPin)
{
    playMelody(buzzerPin, stopMovingMelody, stopMovingMelodyLength);
}

void playTurnLeftTone(int buzzerPin)
{
    playMelody(buzzerPin, turnLeftMelody, turnLeftMelodyLength);
}

void playTurnRightTone(int buzzerPin)
{
    playMelody(buzzerPin, turnRightMelody, turnRightMelodyLength);
}

void playBackwardTone(int buzzerPin)
{
    playMelody(buzzerPin, backwardMelody, backwardMelodyLength);
}

#endif // ROBOT_TONES_H