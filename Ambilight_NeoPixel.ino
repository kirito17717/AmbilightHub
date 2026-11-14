#include <Adafruit_NeoPixel.h>

#define MAX_LEDS 500
#define TIMEOUT_MS 400 // Таймаут прекращения передачи от ПК (мс)

int ledPin = 4;          
uint16_t numLeds = 60;         
Adafruit_NeoPixel *strip = nullptr;

uint8_t ledBuffer[MAX_LEDS * 3];

// Переменные для управления яркостью и плавным розжигом / затуханием
uint8_t currentMasterBrightness = 0; 
uint8_t targetMasterBrightness = 50; // Страховочное значение по умолчанию
unsigned long lastReceiveTime = 0;
bool isConfigured = false;
bool hasFadedOut = true;

// Буфер для поиска заголовка UART побайтово
byte b1 = 0, b2 = 0, b3 = 0;

void setup() {
  Serial.begin(921600);
  Serial.setRxBufferSize(2048);
  delay(500);

  // Инициализация ленты с дефолтными значениями до прихода конфига от ПК
  if (numLeds > 0 && numLeds <= MAX_LEDS) {
    strip = new Adafruit_NeoPixel(numLeds, ledPin, NEO_GRB + NEO_KHZ800);
    strip->begin();
    strip->setBrightness(0);
    strip->show();
  }
}

void loop() {
  unsigned long currentMillis = millis();
  bool frameUpdated = false;

  // Обработка входящих данных по UART с защитой от рассинхрона
  while (Serial.available() > 0) {
    b1 = b2;
    b2 = b3;
    b3 = Serial.read();

    // 1. Пакет конфигурации: 'C', 'f', 'g'
    if (b1 == 'C' && b2 == 'f' && b3 == 'g') {
      if (Serial.available() >= 5) {
        byte hi = Serial.read();
        byte lo = Serial.read();
        byte pin = Serial.read();
        byte brightness = Serial.read();
        byte checksum = Serial.read();

        if (checksum == (hi ^ lo ^ pin ^ brightness ^ 0x33)) {
          uint16_t newTotal = ((hi << 8) & 0xFF00) | (lo & 0x00FF);

          bool pinChanged = (pin != ledPin);
          bool ledsChanged = (newTotal != numLeds);

          if (pinChanged) ledPin = pin;
          if (ledsChanged && newTotal > 0 && newTotal <= MAX_LEDS) numLeds = newTotal;

          // Пересоздаем объект ленты только при реальном изменении конфигурации в ОЗУ
          if ((pinChanged || ledsChanged || strip == nullptr) && numLeds > 0) {
            if (strip != nullptr) {
              delete strip;
            }
            strip = new Adafruit_NeoPixel(numLeds, ledPin, NEO_GRB + NEO_KHZ800);
            strip->begin();
            strip->setBrightness(0);
            strip->show();
          }

          if (brightness > 0) targetMasterBrightness = brightness;
          isConfigured = true;
          lastReceiveTime = currentMillis;
        }
      }
    }
    // 2. Обычный пакет потока кадров Adalight: 'A', 'd', 'a'
    else if (b1 == 'A' && b2 == 'd' && b3 == 'a') {
      if (Serial.available() >= 4) {
        byte hi = Serial.read();
        byte lo = Serial.read();
        byte brightness = Serial.read();
        byte checksum = Serial.read();

        if (checksum == (hi ^ lo ^ brightness ^ 0x55)) {
          uint16_t ledsCount = ((hi << 8) & 0xFF00) | (lo & 0x00FF);
         
          if (brightness > 0) {
            targetMasterBrightness = brightness;
          }

          int bytesToRead = ledsCount * 3;
          unsigned long timeoutStart = millis();
          while (Serial.available() < bytesToRead) {
            if (millis() - timeoutStart > 30) break;
          }

          if (isConfigured && ledsCount <= numLeds && Serial.available() >= bytesToRead) {
            Serial.readBytes(ledBuffer, bytesToRead);

            if (strip != nullptr) {
              for (int i = 0; i < ledsCount; i++) {
                int idx = i * 3;
                strip->setPixelColor(i, ledBuffer[idx], ledBuffer[idx+1], ledBuffer[idx+2]);
              }
              frameUpdated = true;
            }
            lastReceiveTime = currentMillis; 
          }
        }
      }
    }
  }

  // Управление питанием, плавным розжигом, затуханием и таймаутами
  if (isConfigured && numLeds > 0 && strip != nullptr) {
    bool connectionActive = (currentMillis - lastReceiveTime <= TIMEOUT_MS);
    bool brightnessChanged = false;

    if (connectionActive) {
      if (hasFadedOut) {
        currentMasterBrightness = 0;
        hasFadedOut = false;
      }

      if (currentMasterBrightness < targetMasterBrightness) {
        currentMasterBrightness += 3;
        if (currentMasterBrightness > targetMasterBrightness) {
          currentMasterBrightness = targetMasterBrightness;
        }
        brightnessChanged = true;
      }
    } 
    else {
      if (currentMasterBrightness > 0) {
        currentMasterBrightness = (currentMasterBrightness > 8) ? (currentMasterBrightness - 8) : 0;
        brightnessChanged = true;
      }
      
      if (currentMasterBrightness == 0) {
        hasFadedOut = true;
      }
      
      delay(20); 
    }

    // Выводим кадр в ленту только если обновились данные или меняется плавная яркость
    if (frameUpdated || brightnessChanged) {
      strip->setBrightness(currentMasterBrightness);
      strip->show();
    }
  }
}