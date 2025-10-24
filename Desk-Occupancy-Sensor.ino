#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Set the LCD address (commonly 0x27 or 0x3F)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// IR sensor pin
#define IR_SENSOR_PIN 4

void setup() {
  // Initialize LCD
  lcd.init();
  lcd.backlight();

  // Initialize IR sensor pin
  pinMode(IR_SENSOR_PIN, INPUT);

  // Welcome message
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Desk Occupancy");
  lcd.setCursor(0, 1);
  lcd.print("System Ready!");
  delay(2000);
  lcd.clear();
}

void loop() {
  int sensorValue = digitalRead(IR_SENSOR_PIN);

  lcd.setCursor(0, 0);
  lcd.print("Desk Status:     ");

  if (sensorValue == LOW) {
    // Object detected (depends on sensor)
    lcd.setCursor(0, 1);
    lcd.print("  OCCUPIED       ");
  } else {
    lcd.setCursor(0, 1);
    lcd.print("  FREE           ");
  }

  delay(500);
}
