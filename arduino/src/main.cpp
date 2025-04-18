#include <Arduino.h>
#include <IBusBM.h>

IBusBM ibus; // Create iBus object

// Read the number of a given channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue)
{
  uint16_t ch = ibus.readChannel(channelInput);
  if (ch < 100)
    return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

// Read the channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue)
{
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}
void setup()
{
  Serial.begin(115200); // For debugging
  ibus.begin(Serial1);  // Attach iBus to Serial1
}

void loop()
{

  // for (byte i = 1; i < 5; i++)
  // {
  //   int value = readChannel(i, -100, 100, 0);
  //   Serial.print("Ch");
  //   Serial.print(i + 1);
  //   Serial.print(": ");
  //   Serial.print(value);
  //   Serial.print(" | ");
  // }

  // Read all 10 channels (values 1000-2000 μs)
  // for (int i = 6; i < 10; i++)
  // {
  //   uint16_t channelValue = readSwitch(i, false);
  //   Serial.print("CH");
  //   Serial.print(i + 1);
  //   Serial.print(": ");
  //   Serial.print(channelValue);
  //   Serial.print("\t");
  // }

  // Arduino (add to loop())
  Serial.print("{\"ch1\":");
  Serial.print(readChannel(0, -100, 100, 0));
  Serial.print(",\"ch2\":");
  Serial.print(readChannel(1, -100, 100, 0));
  Serial.print(",\"ch3\":");
  Serial.print(readChannel(2, -100, 100, 0));
  Serial.print(",\"ch4\":");
  Serial.print(readChannel(3, -100, 100, 0));
  Serial.print(",\"ch5\":");
  Serial.print(readSwitch(4, false));
  Serial.print(",\"ch6\":");
  Serial.print(readSwitch(5, false));
  Serial.print(",\"ch7\":");
  Serial.print(readSwitch(6, false));
  Serial.print(",\"ch8\":");
  Serial.print(readSwitch(7, false));
  Serial.print(",\"ch9\":");
  Serial.print(readChannel(8, -100, 100, 0));
  Serial.print(",\"ch10\":");
  Serial.print(readChannel(9, -100, 100, 0));
  Serial.println("}");
  Serial.println();
  delay(20); //  50Hz
}
