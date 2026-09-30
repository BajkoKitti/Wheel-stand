#include 
#include 
#include 

// Dedicated XSHUT hardware enable pins for VL53L0X distance sensors
const int xshut1 = 2;
const int xshut2 = 3;
const int xshut3 = 4;

// Runtime I2C addresses assigned dynamically during boot sequencing
const uint8_t address1 = 0x30;
const uint8_t address2 = 0x31;
const uint8_t address3 = 0x32;

VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;

// Modbus RTU node instances for industrial load cell amplifiers
ModbusMaster node1; // Slave ID 1
ModbusMaster node2; // Slave ID 2

// Poll Load Cell 1 (Register 0x0000, 1 register length)
float readLoadCell1(ModbusMaster &node) {
  uint8_t result = node.readHoldingRegisters(0x0000, 1);
  if (result == node.ku8MBSuccess) {
    int16_t val = (int16_t)node.getResponseBuffer(0);
    return (float)val;
  }
  return 0.0;
}

// Poll Load Cell 2 (Register 0x0000, 2 registers length, extracting offset index 1)
float readLoadCell2(ModbusMaster &node) {
  uint8_t result = node.readHoldingRegisters(0x0000, 2);
  if (result == node.ku8MBSuccess) {
    int16_t val = (int16_t)node.getResponseBuffer(1);
    return (float)val;
  }
  return 0.0;
}

void setup() {
  // Initialize USB-CDC debug/data telemetry stream
  Serial.begin(115200);

  // Initialize hardware serial port for RS-485 transceiver
  Serial1.begin(9600);

  // Attach Modbus slave nodes to hardware serial line
  node1.begin(1, Serial1);
  node2.begin(2, Serial1);

  // Initialize I2C bus
  Wire.begin();

  // Configure XSHUT control pins
  pinMode(xshut1, OUTPUT);
  pinMode(xshut2, OUTPUT);
  pinMode(xshut3, OUTPUT);

  // Assert hardware shutdown across all ToF sensors (hold in standby)
  digitalWrite(xshut1, LOW);
  digitalWrite(xshut2, LOW);
  digitalWrite(xshut3, LOW);
  delay(10);

  // Sequentially power up and readdress Laser Sensor 1
  digitalWrite(xshut1, HIGH);
  delay(10);
  sensor1.setTimeout(500);
  if (!sensor1.init()) { while (1); }
  sensor1.setAddress(address1);

  // Sequentially power up and readdress Laser Sensor 2
  digitalWrite(xshut2, HIGH);
  delay(10);
  sensor2.setTimeout(500);
  if (!sensor2.init()) { while (1); }
  sensor2.setAddress(address2);

  // Sequentially power up and readdress Laser Sensor 3
  digitalWrite(xshut3, HIGH);
  delay(10);
  sensor3.setTimeout(500);
  if (!sensor3.init()) { while (1); }
  sensor3.setAddress(address3);
}

void loop() {
  // Read distance measurements from ToF laser sensors (mm)
  uint16_t distance1 = sensor1.readRangeSingleMillimeters();
  uint16_t distance2 = sensor2.readRangeSingleMillimeters();
  uint16_t distance3 = sensor3.readRangeSingleMillimeters();

  // Read force measurements from RS-485 Modbus load cells (N)
  float load1 = readLoadCell1(node1);
  delay(20); // Inter-frame delay to prevent RS-485 bus contention
  float load2 = readLoadCell2(node2);

  unsigned long timestamp = millis();

  // Serialize acquisition telemetry frame into newline-delimited JSON
  Serial.print("{");
  Serial.print("\"timestamp\":");
  Serial.print(timestamp);
  Serial.print(",\"sensor_1\":");
  Serial.print(distance1);
  Serial.print(",\"sensor_2\":");
  Serial.print(distance2);
  Serial.print(",\"sensor_3\":");
  Serial.print(distance3);
  Serial.print(",\"load_1\":");
  Serial.print(load1, 2);
  Serial.print(",\"load_2\":");
  Serial.print(load2, 2);
  Serial.println("}");

  // Loop pacing delay (~10 Hz sampling rate)
  delay(80);
}
