#include <Arduino.h>
#include <M5Unified.h>

// Pin definitions based on StampS3 diagram and README
const int MIDI_TX_PIN = 13; // GPIO13 for MIDI OUT
const int MIDI_RX_PIN = 15; // GPIO15 for MIDI IN
const int PWM_PIN = 39;     // GPIO39 for PWM audio output

// Pin configuration arrays
const int keyPins[13] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14}; // KC to KC+
const byte baseNotes[13] = {60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72}; // C to C

// Control and extension switches
enum SwitchType { OU, OD, CU, CD, S1, S2, S3, NUM_SWITCHES };
const int switchPins[NUM_SWITCHES] = {46, 43, 42, 44, 41, 40, 0}; // OU, OD, CU, CD, S1, S2, S3
const char* switchNames[NUM_SWITCHES] = {"OU", "OD", "CU", "CD", "S1", "S2", "S3"};

// MIDI constants
const byte MIDI_NOTE_ON = 0x90;
const byte MIDI_NOTE_OFF = 0x80;
const byte MIDI_CHANNEL = 0;
const byte VELOCITY = 110;

// State variables
const int NUM_KEYS = 13;
const unsigned long debounceDelay = 50;
int octaveShift = 0; // -5 to +5 octaves
bool audioEnabled = true; // PWM audio on/off state

// Key states
bool keyPressed[NUM_KEYS] = {false};
bool keyLastState[NUM_KEYS];
unsigned long keyDebounceTime[NUM_KEYS] = {0};

// Switch states  
bool switchPressed[NUM_SWITCHES] = {false};
bool switchLastState[NUM_SWITCHES];
unsigned long switchDebounceTime[NUM_SWITCHES] = {0};

// PWM variables
const int PWM_CHANNEL = 0;
const int PWM_RESOLUTION = 4; // 4bit分解能で超低周波数対応

// Last pressed note tracking
int lastPressedNote = -1; // -1 means no note is currently being played

float midiNoteToFrequency(byte note)
{
  float baseFreq = 440.0 * pow(2.0, (note - 69) / 12.0);
  return baseFreq; // tone()関数なら制限なし
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

void startTone(byte note)
{
  if (!audioEnabled) return; // Skip if audio is disabled
  
  float frequency = midiNoteToFrequency(note);
  Serial.print("Starting tone at ");
  Serial.print(frequency);
  Serial.println(" Hz");

  // ArduinoのtoneW関数を使用（低周波数対応）
  tone(PWM_PIN, frequency);
}

void stopTone()
{
  Serial.println("Stopping tone");
  noTone(PWM_PIN);
}

// Function declarations
void handleKey(int keyIndex);
void handleSwitch(int switchIndex);
void initializePins();

void initializePins() {
  // Initialize all key pins
  for (int i = 0; i < NUM_KEYS; i++) {
    pinMode(keyPins[i], INPUT_PULLUP);
    keyLastState[i] = true; // Pull-up default
  }
  
  // Initialize all switch pins
  for (int i = 0; i < NUM_SWITCHES; i++) {
    pinMode(switchPins[i], INPUT_PULLUP);
    switchLastState[i] = true; // Pull-up default
  }
  
  // Configure PWM pin
  pinMode(PWM_PIN, OUTPUT);
}

void setup()
{
  // Initialize M5Unified
  auto cfg = M5.config();
  M5.begin(cfg);

  Serial.begin(115200);
  Serial1.begin(31250, SERIAL_8N1, MIDI_RX_PIN, MIDI_TX_PIN);
  
  initializePins();

  delay(1000); // Wait for serial
  Serial.println("MIDI Keyboard - Refactored with unified switch handling");
  Serial.println("Keys: KC, KC#, KD, KD#, KE, KF, KF#, KG, KG#, KA, KA#, KB, KC+");
  Serial.println("Switches: OU/OD (octave), CU/CD/S1/S2 (extension), S3 (audio toggle)");
  Serial.print("Current octave shift: ");
  Serial.print(octaveShift);
  Serial.print(", Audio: ");
  Serial.println(audioEnabled ? "enabled" : "disabled");
}

void loop()
{
  M5.update(); // Required for M5Unified

  // Handle all switches (octave + extension)
  for (int i = 0; i < NUM_SWITCHES; i++) {
    handleSwitch(i);
  }

  // Handle all keyboard keys
  for (int i = 0; i < NUM_KEYS; i++) {
    handleKey(i);
  }
}

void handleSwitch(int switchIndex) {
  bool currentState = digitalRead(switchPins[switchIndex]);
  
  // Debounce logic
  if (currentState != switchLastState[switchIndex]) {
    switchDebounceTime[switchIndex] = millis();
  }
  
  if ((millis() - switchDebounceTime[switchIndex]) > debounceDelay) {
    if (currentState != switchPressed[switchIndex]) {
      switchPressed[switchIndex] = currentState;
      
      if (!switchPressed[switchIndex]) { // Button pressed (LOW due to pull-up)
        switch (switchIndex) {
          case OU: // Octave Up
            if (octaveShift < 5) {
              octaveShift++;
              Serial.print("Octave Up - Current shift: ");
              Serial.println(octaveShift);
              stopTone();
              lastPressedNote = -1; // Reset last pressed note
            }
            break;
          case OD: // Octave Down
            if (octaveShift > -5) {
              octaveShift--;
              Serial.print("Octave Down - Current shift: ");
              Serial.println(octaveShift);
              stopTone();
              lastPressedNote = -1; // Reset last pressed note
            }
            break;
          case S3: // Audio toggle switch (GPIO0)
            audioEnabled = !audioEnabled;
            Serial.print("Audio ");
            Serial.println(audioEnabled ? "enabled" : "disabled");
            if (!audioEnabled) {
              stopTone(); // Stop current tone if disabling
              lastPressedNote = -1; // Reset last pressed note
            }
            break;
          default: // Other extension switches (CU, CD, S1, S2)
            Serial.print(switchNames[switchIndex]);
            Serial.println(" switch pressed");
            break;
        }
      }
    }
  }
  
  switchLastState[switchIndex] = currentState;
}

void handleKey(int keyIndex) {
  bool currentState = digitalRead(keyPins[keyIndex]);
  
  // Debounce logic
  if (currentState != keyLastState[keyIndex]) {
    keyDebounceTime[keyIndex] = millis();
  }
  
  if ((millis() - keyDebounceTime[keyIndex]) > debounceDelay) {
    if (currentState != keyPressed[keyIndex]) {
      keyPressed[keyIndex] = currentState;
      
      if (!keyPressed[keyIndex]) { // Button pressed (LOW due to pull-up)
        byte midiNote = baseNotes[keyIndex] + (octaveShift * 12);
        // Clamp to valid MIDI range (0-127)
        if (midiNote >= 0 && midiNote <= 127) {
          Serial.print("Key ");
          Serial.print(keyIndex);
          Serial.print(" pressed - MIDI Note ");
          Serial.println(midiNote);
          sendMidiNoteOn(midiNote, VELOCITY);
          lastPressedNote = midiNote; // Remember this as the last pressed note
          startTone(midiNote);
        }
      } else { // Button released (HIGH)
        byte midiNote = baseNotes[keyIndex] + (octaveShift * 12);
        if (midiNote >= 0 && midiNote <= 127) {
          Serial.print("Key ");
          Serial.print(keyIndex);
          Serial.print(" released - MIDI Note ");
          Serial.println(midiNote);
          sendMidiNoteOff(midiNote);
          
          // Only stop tone if this was the last pressed note
          if (midiNote == lastPressedNote) {
            stopTone();
            lastPressedNote = -1; // Reset last pressed note
          }
        }
      }
    }
  }
  
  keyLastState[keyIndex] = currentState;
}

