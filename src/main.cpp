#include <Arduino.h>
#include <M5Unified.h>

// Pin definitions - back to working configuration
const int KC_PIN = 1;       // GPIO1 for KC key (Middle C)
const int MIDI_TX_PIN = 13; // GPIO13 for MIDI OUT
const int PWM_PIN = 39;     // GPIO19 for PWM audio output

// MIDI constants
const byte MIDI_NOTE_ON = 0x90;
const byte MIDI_NOTE_OFF = 0x80;
const byte MIDI_CHANNEL = 0;
const byte KC_NOTE = 60;
const byte VELOCITY = 127;

// Button state
bool kcPressed = false;
bool kcLastState = true;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// PWM variables
const int PWM_CHANNEL = 0;
const int PWM_RESOLUTION = 8;

float midiNoteToFrequency(byte note)
{
  float baseFreq = 440.0 * pow(2.0, (note - 69) / 12.0);
  while (baseFreq < 500.0)
  {
    baseFreq *= 2.0;
  }
  return baseFreq;
}

void sendMidiNoteOn(byte note, byte velocity)
{
  Serial1.write(MIDI_NOTE_ON | MIDI_CHANNEL);
  Serial1.write(note);
  Serial1.write(velocity);
}

void sendMidiNoteOff(byte note)
{
  Serial1.write(MIDI_NOTE_OFF | MIDI_CHANNEL);
  Serial1.write(note);
  Serial1.write(0);
}

void startPWMTone(byte note)
{
  float frequency = midiNoteToFrequency(note);
  Serial.print("Starting PWM tone at ");
  Serial.print(frequency);
  Serial.println(" Hz");

  ledcSetup(PWM_CHANNEL, frequency, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 128);
}

void stopPWMTone()
{
  Serial.println("Stopping PWM tone");
  ledcWrite(PWM_CHANNEL, 0);
}

void setup()
{
  // Initialize M5Unified
  auto cfg = M5.config();
  M5.begin(cfg);

  Serial.begin(115200);
  Serial1.begin(31250, SERIAL_8N1, -1, MIDI_TX_PIN);

  pinMode(KC_PIN, INPUT_PULLUP);
  pinMode(PWM_PIN, OUTPUT);

  delay(1000); // Wait for serial
  Serial.println("MIDI Keyboard Test - KC Button to Note 60");
  Serial.println("Press KC button to send MIDI Note 60");
}

void loop()
{
  bool currentState = digitalRead(KC_PIN);

  if (currentState != kcLastState)
  {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay)
  {
    if (currentState != kcPressed)
    {
      kcPressed = currentState;

      if (!kcPressed)
      {
        Serial.println("KC pressed - Sending MIDI Note ON 60");
        sendMidiNoteOn(KC_NOTE, VELOCITY);
        startPWMTone(KC_NOTE);
      }
      else
      {
        Serial.println("KC released - Sending MIDI Note OFF 60");
        sendMidiNoteOff(KC_NOTE);
        stopPWMTone();
      }
    }
  }

  kcLastState = currentState;
}