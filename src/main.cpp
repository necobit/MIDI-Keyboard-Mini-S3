#include <Arduino.h>
#include <M5Unified.h>

// Pin definitions based on StampS3 diagram and README
const int MIDI_TX_PIN = 13; // GPIO13 for MIDI OUT
const int MIDI_RX_PIN = 15; // GPIO15 for MIDI IN
const int PWM_PIN = 39;     // GPIO39 for PWM audio output

// Keyboard pin definitions (K* keys) - 13-key chromatic keyboard
const int KC_PIN = 1;        // GPIO1 for KC key (Middle C, Note 60)
const int KC_SHARP_PIN = 2;  // GPIO2 for KC# key (Note 61)
const int KD_PIN = 3;        // GPIO3 for KD key (Note 62)
const int KD_SHARP_PIN = 4;  // GPIO4 for KD# key (Note 63)
const int KE_PIN = 5;        // GPIO5 for KE key (Note 64)
const int KF_PIN = 6;        // GPIO6 for KF key (Note 65)
const int KF_SHARP_PIN = 7;  // GPIO7 for KF# key (Note 66)
const int KG_PIN = 8;        // GPIO8 for KG key (Note 67)
const int KG_SHARP_PIN = 9;  // GPIO9 for KG# key (Note 68)
const int KA_PIN = 10;       // GPIO10 for KA key (Note 69)
const int KA_SHARP_PIN = 11; // GPIO11 for KA# key (Note 70)
const int KB_PIN = 12;       // GPIO12 for KB key (Note 71)
const int KC_PLUS_PIN = 14;  // GPIO14 for KC+ key (Note 72, octave up)

// Control buttons
const int OU_PIN = 46; // GPIO46 for Octave Up
const int OD_PIN = 43; // GPIO43 for Octave Down

// Extension switches
const int CU_PIN = 42;  // GPIO42 for CU switch
const int CD_PIN = 44;  // GPIO44 for CD switch  
const int S1_PIN = 41;  // GPIO41 for S1 switch
const int S2_PIN = 40;  // GPIO40 for S2 switch
const int S3_PIN = 0;   // GPIO0 for S3 switch

// MIDI constants
const byte MIDI_NOTE_ON = 0x90;
const byte MIDI_NOTE_OFF = 0x80;
const byte MIDI_CHANNEL = 0;
const byte VELOCITY = 127;

// Keyboard configuration
const int NUM_KEYS = 13;
const int keyPins[NUM_KEYS] = {
    KC_PIN, KC_SHARP_PIN, KD_PIN, KD_SHARP_PIN, KE_PIN, KF_PIN,
    KF_SHARP_PIN, KG_PIN, KG_SHARP_PIN, KA_PIN, KA_SHARP_PIN, KB_PIN, KC_PLUS_PIN};
const byte baseNotes[NUM_KEYS] = {
    60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72 // C to C (one octave)
};

// Button state variables for all keys
bool keyPressed[NUM_KEYS] = {false};
bool keyLastState[NUM_KEYS] = {true, true, true, true, true, true, true,
                               true, true, true, true, true, true}; // Pull-up
unsigned long lastDebounceTime[NUM_KEYS] = {0};
const unsigned long debounceDelay = 50;

// Octave control
int octaveShift = 0; // -5 to +5 octaves
bool ouPressed = false, odPressed = false;
bool ouLastState = true, odLastState = true;
unsigned long ouDebounceTime = 0, odDebounceTime = 0;

// Extension switches state
bool cuPressed = false, cdPressed = false;
bool s1Pressed = false, s2Pressed = false, s3Pressed = false;
bool cuLastState = true, cdLastState = true;
bool s1LastState = true, s2LastState = true, s3LastState = true;
unsigned long cuDebounceTime = 0, cdDebounceTime = 0;
unsigned long s1DebounceTime = 0, s2DebounceTime = 0, s3DebounceTime = 0;

// PWM variables
const int PWM_CHANNEL = 0;
const int PWM_RESOLUTION = 4; // 4bit分解能で超低周波数対応

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
void handleOctaveButtons();
void handleKey(int keyIndex);
void handleExtensionSwitches();

void setup()
{
  // Initialize M5Unified
  auto cfg = M5.config();
  M5.begin(cfg);

  Serial.begin(115200);
  Serial1.begin(31250, SERIAL_8N1, MIDI_RX_PIN, MIDI_TX_PIN);

  // Configure all key pins with pull-up
  for (int i = 0; i < NUM_KEYS; i++)
  {
    pinMode(keyPins[i], INPUT_PULLUP);
  }

  // Configure octave control pins
  pinMode(OU_PIN, INPUT_PULLUP);
  pinMode(OD_PIN, INPUT_PULLUP);
  
  // Configure extension switch pins
  pinMode(CU_PIN, INPUT_PULLUP);
  pinMode(CD_PIN, INPUT_PULLUP);
  pinMode(S1_PIN, INPUT_PULLUP);
  pinMode(S2_PIN, INPUT_PULLUP);
  pinMode(S3_PIN, INPUT_PULLUP);

  // Configure PWM pin
  pinMode(PWM_PIN, OUTPUT);

  delay(1000); // Wait for serial
  Serial.println("MIDI Keyboard - Full 13-key chromatic with tone() function");
  Serial.println("Keys: KC, KC#, KD, KD#, KE, KF, KF#, KG, KG#, KA, KA#, KB, KC+");
  Serial.println("OU/OD: Octave Up/Down controls");
  Serial.println("CU/CD/S1/S2/S3: Extension switches");
  Serial.print("Current octave shift: ");
  Serial.println(octaveShift);
}

void loop()
{
  M5.update(); // Required for M5Unified

  // Handle octave control buttons
  handleOctaveButtons();
  
  // Handle extension switches
  handleExtensionSwitches();

  // Handle all keyboard keys
  for (int i = 0; i < NUM_KEYS; i++)
  {
    handleKey(i);
  }
}

void handleOctaveButtons()
{
  // Handle Octave Up button
  bool ouCurrentState = digitalRead(OU_PIN);
  if (ouCurrentState != ouLastState)
  {
    ouDebounceTime = millis();
  }
  if ((millis() - ouDebounceTime) > debounceDelay)
  {
    if (ouCurrentState != ouPressed)
    {
      ouPressed = ouCurrentState;
      if (!ouPressed && octaveShift < 5)
      { // Button pressed and not at max
        octaveShift++;
        Serial.print("Octave Up - Current shift: ");
        Serial.println(octaveShift);
      }
    }
  }
  ouLastState = ouCurrentState;

  // Handle Octave Down button
  bool odCurrentState = digitalRead(OD_PIN);
  if (odCurrentState != odLastState)
  {
    odDebounceTime = millis();
  }
  if ((millis() - odDebounceTime) > debounceDelay)
  {
    if (odCurrentState != odPressed)
    {
      odPressed = odCurrentState;
      if (!odPressed && octaveShift > -5)
      { // Button pressed and not at min
        octaveShift--;
        Serial.print("Octave Down - Current shift: ");
        Serial.println(octaveShift);
      }
    }
  }
  odLastState = odCurrentState;
}

void handleKey(int keyIndex)
{
  bool currentState = digitalRead(keyPins[keyIndex]);

  // Debounce logic
  if (currentState != keyLastState[keyIndex])
  {
    lastDebounceTime[keyIndex] = millis();
  }

  if ((millis() - lastDebounceTime[keyIndex]) > debounceDelay)
  {
    if (currentState != keyPressed[keyIndex])
    {
      keyPressed[keyIndex] = currentState;

      if (!keyPressed[keyIndex])
      { // Button pressed (LOW due to pull-up)
        byte midiNote = baseNotes[keyIndex] + (octaveShift * 12);
        // Clamp to valid MIDI range (0-127)
        if (midiNote >= 0 && midiNote <= 127)
        {
          Serial.print("Key ");
          Serial.print(keyIndex);
          Serial.print(" pressed - MIDI Note ");
          Serial.println(midiNote);
          sendMidiNoteOn(midiNote, VELOCITY);
          startTone(midiNote);
        }
      }
      else
      { // Button released (HIGH)
        byte midiNote = baseNotes[keyIndex] + (octaveShift * 12);
        if (midiNote >= 0 && midiNote <= 127)
        {
          Serial.print("Key ");
          Serial.print(keyIndex);
          Serial.print(" released - MIDI Note ");
          Serial.println(midiNote);
          sendMidiNoteOff(midiNote);
          stopTone();
        }
      }
    }
  }

  keyLastState[keyIndex] = currentState;
}

void handleExtensionSwitches() {
  // Handle CU switch
  bool cuCurrentState = digitalRead(CU_PIN);
  if (cuCurrentState != cuLastState) {
    cuDebounceTime = millis();
  }
  if ((millis() - cuDebounceTime) > debounceDelay) {
    if (cuCurrentState != cuPressed) {
      cuPressed = cuCurrentState;
      if (!cuPressed) { // Button pressed
        Serial.println("CU switch pressed");
      }
    }
  }
  cuLastState = cuCurrentState;

  // Handle CD switch
  bool cdCurrentState = digitalRead(CD_PIN);
  if (cdCurrentState != cdLastState) {
    cdDebounceTime = millis();
  }
  if ((millis() - cdDebounceTime) > debounceDelay) {
    if (cdCurrentState != cdPressed) {
      cdPressed = cdCurrentState;
      if (!cdPressed) { // Button pressed
        Serial.println("CD switch pressed");
      }
    }
  }
  cdLastState = cdCurrentState;

  // Handle S1 switch
  bool s1CurrentState = digitalRead(S1_PIN);
  if (s1CurrentState != s1LastState) {
    s1DebounceTime = millis();
  }
  if ((millis() - s1DebounceTime) > debounceDelay) {
    if (s1CurrentState != s1Pressed) {
      s1Pressed = s1CurrentState;
      if (!s1Pressed) { // Button pressed
        Serial.println("S1 switch pressed");
      }
    }
  }
  s1LastState = s1CurrentState;

  // Handle S2 switch
  bool s2CurrentState = digitalRead(S2_PIN);
  if (s2CurrentState != s2LastState) {
    s2DebounceTime = millis();
  }
  if ((millis() - s2DebounceTime) > debounceDelay) {
    if (s2CurrentState != s2Pressed) {
      s2Pressed = s2CurrentState;
      if (!s2Pressed) { // Button pressed
        Serial.println("S2 switch pressed");
      }
    }
  }
  s2LastState = s2CurrentState;

  // Handle S3 switch
  bool s3CurrentState = digitalRead(S3_PIN);
  if (s3CurrentState != s3LastState) {
    s3DebounceTime = millis();
  }
  if ((millis() - s3DebounceTime) > debounceDelay) {
    if (s3CurrentState != s3Pressed) {
      s3Pressed = s3CurrentState;
      if (!s3Pressed) { // Button pressed
        Serial.println("S3 switch pressed");
      }
    }
  }
  s3LastState = s3CurrentState;
}