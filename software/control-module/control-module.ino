// References:
// https://electronoobs.com/eng_arduino_tut125.php#google_vignette
// https://arduinogetstarted.com/tutorials/arduino-button-long-press-short-press#google_vignette
// https://www.arduinoslovakia.eu/application/timer-calculator

// 25600 Hz (16000000/((624+1)*1)) -> 10ms -> 100 Hz
// 2560 Hz (16000000/((6249+1)*1)) -> 100ms
// 256 Hz (16000000/((62499+1)*1)) -> 1000ms -> 1 Hz

// 256 Hz -> 1000ms -> 1 Hz
// 512 Hz -> 500ms - 2Hz
// 1024 Hz -> 250ms -> 4Hz
// 2048 Hz -> 125ms -> 8Hz

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

LiquidCrystal_I2C lcd(0x3f, 20, 4); // Set the LCD address
int currentLayer = 0; // 0 for main menu, 1 for adjusting settings
int currentSetting = 0; // Tracks which setting is currently being adjusted
int settingsValue[4] = {0, 0, 0, 0}; // Stores values for each setting (pwm, width, delay and amp)

int adjustmentIncrement = 0; // Tracks incremental changes for adjustments

// Custom arrow character (to pointing to a desired setting)
byte arrow[8] = {
  0b00000,  // -----
  0b00100,  // ---*-
  0b00110,  // ---**
  0b11111,  // *****
  0b00110,  // ---**
  0b00100,  // ---*-
  0b00000,  // -----
  0b00000   // -----
};

byte arrowBack[8] = {
  0b00100,  //   *
  0b01110,  //  ***
  0b11111,  // *****
  0b00100,  //   *
  0b00100,  //   *
  0b00111,  //   ***
  0b00000,  //  
  0b00000   //  
};

#define Clock 5   // Clock pin connected to D9
#define Data 6    // Data pin connected to D8
#define Push 7    // Push button pin connected to D7

int counter = 0;                    // Use this variable to store "steps"
int currentStateClock;              // Store the status of the clock pin (HIGH or LOW)
int lastStateClock;                 // Store the PREVIOUS status of the clock pin (HIGH or LOW)
String currentDir ="";              // Use this to print text 
unsigned long lastButtonPress = 0;  // Use this to store if the push button was pressed or not

const int SHORT_PRESS_TIME = 50; // 500 milliseconds for a short press
const int LONG_PRESS_TIME = 500; // 1000 milliseconds for a long press

unsigned long buttonPressTime = 0;  // When the button was pressed
bool isPressing = false;
bool isLongDetected = false;

#define speakerPin 9  // ON/OFF
#define ledPin 4 // Strobe
#define signalPin 11  // Amplitude pwm

#define freqPin A1 // Frequency potentiometer    

volatile int divider = 0;
volatile int dutyCycle = 0;

unsigned long tMax = 0;
unsigned long calcTMax = 0;

int pwmValue = 0; // Zmienna do przechowywania wartości z potencjometru (PWM)
int freqValue = 0; // Zmienna do przechowywania wartości z potencjometru (częstotliwość)
int ampValue = 0;

int lastPwmValue = 0; // Ostatnio zapisana wartość PWM
int lastFreqValue = 0; // Ostatnio zapisana wartość częstotliwości
int lastAmpValue = 0;

unsigned long displayFrequency = 0; // Stores the frequency to be displayed

unsigned long previousMillis = 0;  // will store last time LED was updated

void setupDisplay() {
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, arrow);
  lcd.createChar(1, arrowBack);

  // Draw the menu items
  lcd.setCursor(2, 0); lcd.print("PWM");
  lcd.setCursor(2, 1); lcd.print("Pulse Width");
  lcd.setCursor(2, 2); lcd.print("Pulse Delay");
  lcd.setCursor(2, 3); lcd.print("Amplitude");
}

void updateDisplay() {
    static int lastSetting = -1;
    static int lastLayer = -1;
    static int lastValue = 10; // To detect changes in value for the current setting (change to -1 if not working)
    static unsigned long lastFrequency = 0; // To detect changes in frequency

    // Check if there's a need to update the display
    if (currentLayer != lastLayer || currentSetting != lastSetting || (currentLayer == 1 && settingsValue[currentSetting] != lastValue) || displayFrequency != lastFrequency) {
        lcd.clear();

        if (currentLayer == 0) {
            // Main menu display logic
            lcd.setCursor(0, 0); lcd.print(" PWM");
            lcd.setCursor(0, 1); lcd.print(" Strob Width");
            lcd.setCursor(0, 2); lcd.print(" Strob Delay");
            lcd.setCursor(0, 3); lcd.print(" Amplitude");

            // Move the custom arrow to the current setting
            lcd.setCursor(0, currentSetting); lcd.write(byte(0));  // arrow

            // Display frequency on the right
            lcd.setCursor(15, 0); // Set cursor position for frequency display
            lcd.print("Freq:");
            lcd.setCursor(15, 1); 
            lcd.print(displayFrequency); 
            lcd.print("Hz");
        } else if (currentLayer == 1) {
            // Adjustment layer display logic
            lcd.setCursor(0, 0); lcd.write(byte(1));  // back arrow
            lcd.setCursor(1, 0);
            switch(currentSetting) {
                case 0: lcd.print("PWM:"); break;
                case 1: lcd.print("Strob Width:"); break;
                case 2: lcd.print("Strob Delay:"); break;
                case 3: lcd.print("Amplitude:"); break;
            }
            lcd.setCursor(1, 1);
            lcd.print(settingsValue[currentSetting]); // Display the current value of the setting

            lcd.print("%"); // Assuming settings are represented as percentages
                        // Display frequency on the right
            lcd.setCursor(15, 0); // Set cursor position for frequency display
            lcd.print("Freq: ");
            lcd.setCursor(15, 1); 
            lcd.print(displayFrequency); 
            lcd.print("Hz");
        }

        // Remember the last state to minimize unnecessary updates
        lastLayer = currentLayer;
        lastSetting = currentSetting;
        lastValue = settingsValue[currentSetting];
        lastFrequency = displayFrequency; // Remember the last frequency
    }
}

void setupEncoder() {
  pinMode(Clock, INPUT_PULLUP);
  pinMode(Data, INPUT_PULLUP);
  pinMode(Push, INPUT_PULLUP);
  lastStateClock = digitalRead(Clock);
}

void updateEncoder() {
    currentStateClock = digitalRead(Clock);

    if (currentStateClock != lastStateClock) {
        if (currentStateClock == HIGH) {
            if (currentLayer == 0) {
                // Navigation in the main menu
                if (digitalRead(Data) != currentStateClock) {
                    currentSetting--;
                    if (currentSetting < 0) currentSetting = 3; // Wrap around to the last setting
                } else {
                    currentSetting++;
                    if (currentSetting > 3) currentSetting = 0; // Wrap around to the first setting
                }
            } else if (currentLayer == 1) {
                // Adjusting the current setting's value in the second layer
                if (digitalRead(Data) != currentStateClock) {
                    settingsValue[currentSetting] = max(settingsValue[currentSetting] - 1, 0);
                } else {
                    settingsValue[currentSetting] = min(settingsValue[currentSetting] + 1, 100);
                }
            }
        }
        lastStateClock = currentStateClock;
    }
    // Button press handling
    int btnState = digitalRead(Push);
    static unsigned long buttonDownTime = 0; // Timestamp when the button was pressed
    static bool buttonPressed = false;       // Indicates if the button is currently being pressed

    if (btnState == LOW) {
        if (!buttonPressed) {
            buttonDownTime = millis(); // Record the time when the button was pressed
            buttonPressed = true;
        }
    } else if (btnState == HIGH && buttonPressed) {
        unsigned long pressDuration = millis() - buttonDownTime;
        buttonPressed = false; // Reset the button press state for the next detection

        if (pressDuration >= LONG_PRESS_TIME) {
            // Long press action
            if (currentLayer == 1) {
                // Serial.println("Settings saved (Layer 1, Long Press).");
                saveSetting(currentSetting, settingsValue[currentSetting]);

                // Directly show "Saved" message on the display
                lcd.setCursor(0, 3); // Set the cursor to the bottom line or any desired position
                lcd.print("Saved to memory     "); // Print "Saved" and clear the rest of the line
            }
        } else if (pressDuration >= SHORT_PRESS_TIME && pressDuration < LONG_PRESS_TIME) {
            // Short press action - Toggle between layers
            currentLayer = !currentLayer;
            // Serial.println("Layer toggled by short press.");
        }
    }
}

void saveSetting(int settingIndex, int value) {
  int eepromAddress = settingIndex * 2; // Calculate starting address for the setting
  EEPROM.put(eepromAddress, value);
}

void loadSettings() {
  for (int i = 0; i < 4; i++) {
    int eepromAddress = i * 2; // Calculate starting address for the setting
    int value = 0;
    EEPROM.get(eepromAddress, value);
    settingsValue[i] = value;
  }
}

void setupTimer1() {
  noInterrupts();
  // Clear registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  
  OCR1A = 65535; 
  // CTC
  TCCR1B |= (1 << WGM12);
  // Prescaler 1
  TCCR1B |= (1 << CS10);
  // Output Compare Match A Interrupt Enable
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}

void setup() {
  Serial.begin(115200);
  pinMode(speakerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(signalPin, OUTPUT);
  setupTimer1();
  
  setupEncoder();  // Initialize encoder setup
  setupDisplay();  // Initialize lcd setup
  loadSettings();  // Load saved settings from EPROM
}

void adjustAmplitude() {
    // Map the stored Amplitude percentage (0-100%) to PWM range (0-255)
    int pwmAmplitude = map(settingsValue[3], 0, 100, 0, 255);
    analogWrite(signalPin, pwmAmplitude); // Adjust the amplitude
}

void adjustPWM() {
    // Map the stored PWM percentage (0-100%) to duty cycle range (0-255)
    dutyCycle = map(settingsValue[0], 0, 100, 0, 255);
}

// Define a struct to hold both OCR1A values and the corresponding prescaler setting for easy lookup.
struct FrequencySetting {
    unsigned int ocr1a;
    byte prescalerBits;
};

// Initialize the lookup table with OCR1A values and prescaler settings for frequencies 1Hz to 100Hz.
const FrequencySetting frequencyLookupTable[101] = {
    {65535, 0b000}, // 0 Hz, timer zatrzymany
    {62499, 0b001}, // 256 Hz
    {31249, 0b001}, // 512 Hz
    {20832, 0b001}, // ~768 Hz
    {15624, 0b001}, // 1024 Hz
    {12499, 0b001}, // 1280 Hz
    {10415, 0b001}, // ~1536 Hz
    {8927, 0b001},  // ~1792 Hz
    {7811, 0b001},  // ~2048 Hz
    {6943, 0b001},  // ~2304 Hz
    {6249, 0b001},  // 2560 Hz
    {5680, 0b001},  // ~2816 Hz
    {5207, 0b001},  // ~3072 Hz
    // ...
    {4806, 0b001},  // ~3328 Hz
    {4463, 0b001},  // ~3584 Hz
    {4165, 0b001},  // ~3840 Hz
    {3905, 0b001},  // ~4096 Hz
    {3675, 0b001},  // ~4352 Hz
    {3471, 0b001},  // ~4608 Hz
    {3288, 0b001},  // ~4864 Hz
    {3124, 0b001},  // 5120 Hz
    {2975, 0b001},  // ~5376 Hz
    {2839, 0b001},  // ~5633 Hz
    {2716, 0b001},  // ~5888 Hz
    {2603, 0b001},  // ~6144 Hz
    {2499, 0b001},  // 6400 Hz
    {2402, 0b001},  // ~6658 Hz
    {2313, 0b001},  // ~6914 Hz
    {2231, 0b001},  // ~7168 Hz
    {2154, 0b001},  // ~7424 Hz
    {2082, 0b001},  // ~7681 Hz
    {2015, 0b001},  // ~7936 Hz
    {1952, 0b001},  // ~8192 Hz
    {1892, 0b001},  // ~8452 Hz
    {1837, 0b001},  // ~8705 Hz
    {1784, 0b001},  // ~8963 Hz
    {1735, 0b001},  // ~9216 Hz
    {1688, 0b001},  // ~9473 Hz
    {1643, 0b001},  // ~9732 Hz
    {1601, 0b001},  // ~9987 Hz
    {1561, 0b001},  // ~10243 Hz
    {1523, 0b001}, // ~10498 Hz
    {1487, 0b001}, // ~10752 Hz
    {1452, 0b001}, // ~11011 Hz
    {1419, 0b001}, // ~11267 Hz
    {1387, 0b001}, // ~11527 Hz
    {1357, 0b001}, // ~11782 Hz
    {1328, 0b001}, // ~12039 Hz
    {1301, 0b001}, // ~12288 Hz
    {1274, 0b001}, // ~12549 Hz
    {1249, 0b001}, // 12800 Hz
    {1224, 0b001}, // ~13061 Hz
    {1200, 0b001}, // ~13322 Hz
    {1178, 0b001}, // ~13570 Hz
    {1156, 0b001}, // ~13828 Hz
    {1135, 0b001}, // ~14084 Hz
    {1115, 0b001}, // ~14336 Hz
    {1095, 0b001}, // ~14598 Hz
    {1076, 0b001}, // ~14856 Hz
    {1058, 0b001}, // ~15108 Hz
    {1040, 0b001}, // ~15369 Hz
    {1023, 0b001}, // 15625 Hz
    {1007, 0b001}, // ~15873 Hz
    {991, 0b001},  // ~16129 Hz
    {975, 0b001},  // ~16393 Hz
    {960, 0b001},  // ~16649 Hz
    {945, 0b001},  // ~16913 Hz
    {931, 0b001},  // ~17167 Hz
    {918, 0b001},  // ~17410 Hz
    {904, 0b001},  // ~17679 Hz
    {891, 0b001},  // ~17937 Hz
    {879, 0b001},  // ~18181 Hz
    {867, 0b001},  // ~18433 Hz
    {855, 0b001},  // ~18691 Hz
    {843, 0b001},  // ~18957 Hz
    {832, 0b001},  // ~19207 Hz
    {821, 0b001},  // ~19464 Hz
    {810, 0b001},  // ~19728 Hz
    {800, 0b001},  // ~19975 Hz
    {790, 0b001},  // ~20227 Hz
    {780, 0b001},  // ~20486 Hz
    {770, 0b001},  // ~20752 Hz
    {761, 0b001},  // ~20997 Hz
    {752, 0b001},  // ~21248 Hz
    {743, 0b001},  // ~21505 Hz
    {734, 0b001},  // ~21768 Hz
    {725, 0b001},  // ~22038 Hz
    {717, 0b001},  // ~22284 Hz
    {709, 0b001},  // ~22535 Hz
    {701, 0b001},  // ~22792 Hz
    {693, 0b001},  // ~23054 Hz
    {685, 0b001},  // ~23323 Hz
    {678, 0b001},  // ~23564 Hz
    {671, 0b001},  // ~23809 Hz
    {663, 0b001},  // ~24096 Hz
    {656, 0b001},  // ~24353 Hz
    {650, 0b001},  // ~24577 Hz
    {643, 0b001},  // ~24844 Hz
    {636, 0b001},  // ~25117 Hz
    {630, 0b001},  // ~25356 Hz
    {624, 0b001},  // 25600 Hz
};

void adjustFrequency() {
    // Odczyt wartości z potencjometru podłączonego do freqPin
    freqValue = analogRead(freqPin);
    // Sprawdzenie, czy zmiana wartości przekracza próg 5 (zabezpiecza przed niepożądanymi odczytami)
    if (abs(freqValue - lastFreqValue) > 5) {
    // Mapowanie wartości potencjometru na indeks częstotliwości unsigned int frequencyIndex = map(freqValue, 0, 1023, 0, 100);
    // Pobranie nowych wartości OCR1A i preskalera z tablicy ustawień częstotliwości
    unsigned int newOCR1A = frequencyLookupTable[frequencyIndex].ocr1a;
    byte newPrescalerBits = frequencyLookupTable[frequencyIndex].prescalerBits;
    // Aktualizacja nowych wartości OCR1A i preskalera w rejestrach timera
    OCR1A = newOCR1A;
    TCCR1B = (TCCR1B & 0b11111000) | newPrescalerBits;
    // Obliczanie rzeczywistej częstotliwości na podstawie OCR1A
    unsigned long frequency = 16000000 / (OCR1A + 1);
    // Mapowanie częstotliwości timera na właściwy zakres wyświetlania (1-100 Hz)
    displayFrequency = map(frequency, 256, 25600, 1, 100);
    // Obliczanie okresu sygnału w mikrosekundach
    tMax = (1000000 / frequency);
    // Mapowanie okresu sygnału na właściwy zakres (10-1000 ms)
    calcTMax = map(tMax, 39, 3906, 10, 1000);
    // Zapisywanie bieżącej wartości potencjometru
    lastFreqValue = freqValue;
    } 
}

void loop() {
  adjustAmplitude();
  adjustPWM();
  adjustFrequency();

  unsigned int pulseWidth = map(settingsValue[1], 0, 100, 0, calcTMax);
  unsigned int pulseDelay = map(settingsValue[2], 0, 100, 0, (calcTMax - pulseWidth));
  
  // Warunki dla diody LED:
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= pulseDelay + pulseWidth) {
    digitalWrite(ledPin, LOW);
  } else if (currentMillis - previousMillis >= pulseDelay) {
    digitalWrite(ledPin, HIGH);
  }

  updateEncoder(); // Continuously read and update encoder state
  updateDisplay(); // Continuously read and update lcd state
}

ISR(TIMER1_COMPA_vect) {  // interrupt service routine (ISR)
  //digitalWrite(13, digitalRead(13)^1);
  divider++;
  if(divider > 255) { // wartosc o która spowalniamy
    digitalWrite(speakerPin, HIGH);
    divider = 0;
    previousMillis = millis();
  }
  else if(divider == dutyCycle) { // szerokosc wypelnienia: dla 127 -> 50%
    digitalWrite(speakerPin, LOW);
  }
}