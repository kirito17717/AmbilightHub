#include <FastLED.h>

#define DATA_PIN     4      // Пин данных (GPIO)
#define LED_TYPE     WS2812B
#define COLOR_ORDER  GRB

CRGB* leds = nullptr;
uint16_t totalLeds = 0;

#define UART_BAUD   921600
#define TIMEOUT_MS  400   // Таймаут прекращения передачи от ПК

unsigned long lastReceiveTime = 0;
bool isConfigured = false;

// Переменные для управления яркостью и плавным розжигом
uint8_t currentMasterBrightness = 0; 
uint8_t targetMasterBrightness = 50; 
bool hasFadedOut = true;

void setup() {
  Serial.begin(UART_BAUD);
  Serial.setRxBufferSize(2048);
}

void loop() {
  unsigned long currentMillis = millis();

  // Обработка входящих данных по UART
  if (Serial.available() >= 3) {
    byte b1 = Serial.read();
    byte b2 = Serial.read();
    byte b3 = Serial.read();

    // 1. Пакет конфигурации: 'C', 'f', 'g'
    if (b1 == 'C' && b2 == 'f' && b3 == 'g') {
      if (Serial.available() >= 4) {
        byte hi = Serial.read();
        byte lo = Serial.read();
        byte brightness = Serial.read();
        byte checksum = Serial.read();

        if (checksum == (hi ^ lo ^ brightness ^ 0x33)) {
          uint16_t newTotal = ((hi << 8) & 0xFF00) | (lo & 0x00FF);

          if (newTotal > 0 && newTotal != totalLeds) {
            if (leds != nullptr) {
              delete[] leds;
            }
            leds = new CRGB[newTotal];
            totalLeds = newTotal;
            fill_solid(leds, totalLeds, CRGB::Black);
            
            FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, totalLeds).setCorrection(TypicalLEDStrip);
          }

          if (brightness > 0) targetMasterBrightness = brightness;
          isConfigured = true;
          lastReceiveTime = currentMillis;
          
          Serial.write("OK");
        }
      }
    }
    // 2. Обычный пакет потока кадров Adalight: 'A', 'd', 'a'
    else if (b1 == 'A' && b2 == 'd' && b3 == 'a') {
      if (isConfigured && totalLeds > 0 && Serial.available() >= 4) {
        byte hi = Serial.read();
        byte lo = Serial.read();
        byte brightness = Serial.read();
        byte checksum = Serial.read();

        if (checksum == (hi ^ lo ^ brightness ^ 0x55)) {
          uint16_t ledsCount = ((hi << 8) & 0xFF00) | (lo & 0x00FF);
         
          // Обновляем целевую яркость только если ПК прислал валидное ненулевое значение
          if (brightness > 0) {
            targetMasterBrightness = brightness;
          }

          int bytesToRead = ledsCount * 3;
          unsigned long timeoutStart = millis();
          while (Serial.available() < bytesToRead) {
            if (millis() - timeoutStart > 30) break;
          }

          if (ledsCount <= totalLeds) {
            for (int i = 0; i < ledsCount; i++) {
              byte r = Serial.read();
              byte g = Serial.read();
              byte b = Serial.read();
              leds[i] = CRGB(r, g, b);
            }
            lastReceiveTime = currentMillis; 
          }
        }
      }
    }
  }

  // Управление питанием, плавным розжигом и таймаутами
  if (isConfigured && totalLeds > 0) {
    bool connectionActive = (currentMillis - lastReceiveTime <= TIMEOUT_MS);

    if (connectionActive) {
      // Если лента до этого полностью погасла — разрешаем плавный розжиг
      if (hasFadedOut) {
        currentMasterBrightness = 0;
        hasFadedOut = false;
      }

      // Плавное увеличение яркости до целевой (розжиг займет около 1 секунды)
      if (currentMasterBrightness < targetMasterBrightness) {
        currentMasterBrightness = qadd8(currentMasterBrightness, 3); // Шаг инкремента
        if (currentMasterBrightness > targetMasterBrightness) {
          currentMasterBrightness = targetMasterBrightness;
        }
      }
    } 
    else {
      // Таймаут (ПК выключен / данные не идут): плавно гасим общую яркость в 0
      if (currentMasterBrightness > 0) {
        currentMasterBrightness = qsub8(currentMasterBrightness, 8); // Скорость затухания
      }
      
      // Как только яркость полностью упала до 0, фиксируем состояние для нового розжига
      if (currentMasterBrightness == 0) {
        hasFadedOut = true;
      }
      
      delay(20); // Разгружаем процессор только при отсутствии связи
    }

    // Применяем текущую яркость и выводим кадр в ленту
    FastLED.setBrightness(currentMasterBrightness);
    FastLED.show();
  }
}
