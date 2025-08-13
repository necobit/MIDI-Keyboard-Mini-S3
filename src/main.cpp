#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include "soc/usb_serial_jtag_reg.h"
#include "soc/system_reg.h"
#include "hal/usb_serial_jtag_ll.h"

// USB MIDI object
Adafruit_USBD_MIDI usb_midi;

void forceUSBDeviceMode() {
  // Disable USB Serial JTAG to free up USB peripheral
  usb_serial_jtag_ll_disable_intr_mask(USB_SERIAL_JTAG_INTR_SERIAL_IN_EMPTY |
                                       USB_SERIAL_JTAG_INTR_SERIAL_OUT_RECV_PKT);
  
  // Reset USB Serial JTAG
  REG_SET_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_USB_DEVICE_RST);
  REG_CLR_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_USB_DEVICE_RST);
  
  Serial.println("USB JTAG disabled, switching to USB device mode");
}

void setup()
{
  delay(100);
  
  // Force USB device mode early
  forceUSBDeviceMode();
  
  // Configure USB descriptors before initialization
  TinyUSBDevice.setManufacturerDescriptor("ESP32-S3");
  TinyUSBDevice.setProductDescriptor("MIDI Keyboard");
  TinyUSBDevice.setSerialDescriptor("123456");
  
  // Initialize USB MIDI
  usb_midi.setStringDescriptor("ESP32-S3 MIDI");
  usb_midi.begin();
  
  delay(1000);
  
  Serial.begin(115200);
  delay(100);

  Serial.println("USB MIDI device mode forced");
  Serial.println("Device should now appear as MIDI Keyboard");
}

unsigned long previousTime = 0;
uint8_t noteCounter = 60;

void loop()
{
  unsigned long currentTime = millis();

  // Send a test note every 3 seconds
  if (currentTime - previousTime > 3000)
  {
    // Check if USB device is mounted
    if (TinyUSBDevice.mounted())
    {
      Serial.print("USB mounted - Sending MIDI note: ");
      Serial.println(noteCounter);

      // Send raw MIDI bytes instead of using high-level methods
      uint8_t note_on[4] = {0x09, 0x90, noteCounter, 127}; // Cable 0, Note On
      usb_midi.write(note_on, 4);

      delay(500);

      uint8_t note_off[4] = {0x08, 0x80, noteCounter, 64}; // Cable 0, Note Off
      usb_midi.write(note_off, 4);

      noteCounter++;
      if (noteCounter > 72)
        noteCounter = 60;
    }
    else
    {
      Serial.println("USB not mounted yet - waiting for host connection...");
    }

    previousTime = currentTime;
  }

  delay(10);
}