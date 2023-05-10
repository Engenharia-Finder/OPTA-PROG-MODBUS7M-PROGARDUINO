/**
  Energy Management with Opta™ - Application Note
  Name: energy_management
  Purpose: Based on the power profile and state, manage the power distribution for devices of interest.

  @author Arduino
  @version 1.0 01/20/2023 
*/

#include "stm32h7xx_ll_gpio.h"
#include "thingProperties.h"

#include <ArduinoModbus.h>
#include <ArduinoRS485.h>
#include <Scheduler.h>

constexpr auto baudrate { 19200 };

// Calculate preDelay and postDelay in microseconds as per Modbus RTU Specification
// MODBUS over serial line specification and implementation guide V1.02
// Paragraph 2.5.1.1 MODBUS Message RTU Framing
// https://modbus.org/docs/Modbus_over_serial_line_V1_02.pdf
constexpr auto bitduration { 1.f / baudrate };
constexpr auto preDelayBR { bitduration * 9.6f * 3.5f * 1e6 };
constexpr auto postDelayBR { bitduration * 9.6f * 3.5f * 1e6 };

#define F7M24 0x21

#define pause_trigger 15
int start_time = 0;
int rs485_counter = 0;
int counter = 0;

// Device default state definition
int Device_1 = 0;
int Device_2 = 0;

// Device state flag
uint8_t Device_1_f = 0;
uint8_t Device_2_f = 0;

// Global Parameters
uint8_t a1_state = 0;
int32_t V_actual, V_avg, V_max, V_min;
int32_t A_actual, A_avg, A_max, A_min;
int32_t W_actual, W_avg, W_max, W_min;
int32_t Var_actual, Var_avg, Var_max, Var_min;
int32_t Va_actual, Va_avg, Va_max, Va_min;
int32_t Wh_packet, Varh_packet;

typedef struct uPWR_STRUCT {
  float uV_code;
  float uW_code;
  float uWh_code;
} PWR_STRUCT;
PWR_STRUCT user_profile;

/*************************************
* Energy Meter related routines
*************************************/

/**
  Control the relay output based on the user (consumption) profile input and configured power/energy target.

  @param desired_target Desired resource required to run the connected device on the relay.
  @param req_target Minimum resource required to run the connected device on the relay.
  @param relayTarget Relay to activate or deactivate.
  @return Returns 0 or 1, representing HIGH state for 1 and LOW state for 0.
*/
uint8_t relay_Trigger(int desired_target, int req_target, pin_size_t relayTarget){
  if ((desired_target >= req_target) && (desired_target < (req_target*user_profile.uV_code))){
    digitalWrite(relayTarget, HIGH);
    Serial.println(F("Energy Manager: Stable operation margin - Turning ON: "));
    Serial.print(relayTarget);
    Serial.println(F(""));
    return 1;
  } else {
    digitalWrite(relayTarget, LOW);
    Serial.println(F("Energy Manager: Unstable / possible overload - Turning OFF: "));
    Serial.print(relayTarget);
    Serial.println(F(""));
    return 0;
  }
}

/**
  Initial user profile setup.

  @param init_OperMargin  System operation margin for power budget represented in percentage.
  @param init_Watt  User defined Wattage limit for the system.
  @param init_WhCon User defined Energy consumption limit for the system.
*/
void consumption_profile(uint32_t init_OperMargin, uint32_t init_Watt, uint32_t init_WhCon){
  uOperMargin = init_OperMargin;
  user_profile.uV_code = uOperMargin;

  uWatt = init_Watt;
  user_profile.uW_code = uWatt;

  uWhCon = init_WhCon;
  user_profile.uWh_code = uWhCon;
}

/**
  Simple consumption profile value updater.
*/
void consumption_var_container(){
  uOperMargin = user_profile.uV_code;
  uWatt = user_profile.uW_code;
  uWhCon = user_profile.uWh_code;
}

/**
  Monitors and uses the user defined profile and retrieved information from Energy meter to manage connected devices of interest.

  @param Wh_packet Retrieved energy information from Energy meter in unit of [Wh].
  @param Device_#_f Device flag controlled by relay_Trigger() function. # specifies device number or designation.
  @param directOverride1 Direct override flag for Device #1 controlled via Cloud.
  @param W_avg Average power information retrieved from Energy meter. 
*/
void energy_distro_ctrl(){
  if (Wh_packet != 0){
    uWhOpta = Wh_packet;
  } else {
    Serial.println(F("Energy Manager: Energy information recollection stand-by"));
  }

  // Energy consumption conditionar with 10% safety margin
  if ((Wh_packet*user_profile.uV_code) <= user_profile.uWh_code){
    // Device #1 specific behaviors
    Device_1_f = relay_Trigger(1, A_actual, D0);
    if (!Device_1_f){
      if (relay_Trigger(3, W_actual, D0)){
        Serial.println(F("Energy Manager: Secondary condition pass, may be unstable"));          
      } else {
        Serial.println(F("Energy Manager: Insufficient resource"));
        digitalWrite(D0, LOW); 
      }
    } else{
      Serial.println(F("Energy Manager: Conditions green"));
    }

    // Device #2 specific behaviors
    Device_2_f = relay_Trigger(12, W_avg, D1);
    if (!Device_2_f){
      if (relay_Trigger(12, W_actual, D1)){
        Serial.println(F("Energy Manager: Secondary Power condition pass"));    
      } else {
        Serial.println(F("Energy Manager: Insufficient resource"));
        digitalWrite(D0, LOW); 
      }
    } else{
      Serial.println(F("Energy Manager: Conditions green"));
    }

  } else {
    digitalWrite(D0, LOW); 
    digitalWrite(D1, LOW); 
    Serial.println(F("Energy Manager: Energy consumption is high! - Warning"));
  }
  
  // Direct override request for Device #1
  if (directOverride1 == true){
    // Override Device #1
    Serial.println(F("Energy Manager: Direct Override Request Received"));
    digitalWrite(D0, HIGH); 
  }

  // Conditioner to notify users without cutting power to electronic devices
  if ((W_avg*user_profile.uV_code) > user_profile.uW_code){
    Serial.println(F("Energy Manager: Average Power is above profile limit! - Warning"));
    Serial.print(W_avg*user_profile.uV_code);
    Serial.println(F(""));
  }
}

/**
  Displays electrical, power, and energy consumption information retrieved from the energy meter.

  @param V_XXX Voltage information from Energy meter categorized in actual, average, maximum, and minimum value.
  @param A_XXX Current information from Energy meter categorized in actual, average, maximum, and minimum value.
  @param W_XXX Power information from Energy meter categorized in actual, average, maximum, and minimum value.
  @param Var_XXX Reactive power total information from Energy meter categorized in actual, average, maximum, and minimum value.
  @param Va_XXX Apparent power total information from Energy meter categorized in actual, average, maximum, and minimum value.
  @param Wh_packet Power based energy information from Energy meter.
  @param Varh_packet Reactive power total based energy information from Energy meter.
*/
void modbus_com_monitor(){
  Serial.println(F("||| Energy  |||||||||||||||||||||"));
  Serial.println(Wh_packet);
  Serial.println(Varh_packet);

  Serial.println(F("||| Act. Data  ||||||||||||||||||"));
  Serial.println(V_actual);
  Serial.println(A_actual);
  Serial.println(W_actual);
  Serial.println(Var_actual);
  Serial.println(Va_actual);

  Serial.println(F("||| AVG Data  |||||||||||||||||||"));
  Serial.println(V_avg);
  Serial.println(A_avg);
  Serial.println(W_avg);
  Serial.println(Var_avg);
  Serial.println(Va_avg);

  Serial.println(F("||| Max. Data  ||||||||||||||||||"));
  Serial.println(V_max);
  Serial.println(A_max);
  Serial.println(W_max);
  Serial.println(Var_max);
  Serial.println(Va_max);

  Serial.println(F("||| Min. Data  ||||||||||||||||||"));
  Serial.println(V_min);
  Serial.println(A_min);
  Serial.println(W_min);
  Serial.println(Var_min);
  Serial.println(Va_min);

  Serial.println(F("||| Cloud Parameter Data  |||||||"));
  Serial.println(user_profile.uV_code);
  Serial.println(user_profile.uW_code);
  Serial.println(user_profile.uWh_code);
}

/**
  Requests and retrieves actual electrical, and power information from Energy meter over Modbus RTU protocol.
*/
void modbus_com_actual(){
  // Actual Measurements
  // Voltage (V)
  V_actual = readInputRegisterValues(F7M24, 0x6B, 2);

  // Current (A)
  A_actual = readInputRegisterValues(F7M24, 0x7E, 2);

  // Active Power Total - Pt (W)
  W_actual = readInputRegisterValues(F7M24, 0x8C, 2);
  
  // Reactive Power Total - Qt (var)
  Var_actual = readInputRegisterValues(F7M24, 0x94, 2);

  // Apparent Power Total - St (VA)
  Va_actual = readInputRegisterValues(F7M24, 0x9C, 2);
  delay(100);
}

/**
  Requests and retrieves average electrical, and power information from Energy meter over Modbus RTU protocol.
*/
void modbus_com_avg(){
  // Average Measurements
  // Voltage (V)
  V_avg = readInputRegisterValues(F7M24, 0x1FB, 2);

  // Current (A)
  A_avg = readInputRegisterValues(F7M24, 0x20E, 2);

  // Active Power Total - Pt (W)
  W_avg = readInputRegisterValues(F7M24, 0x21C, 2);
  
  // Reactive Power Total - Qt (var)
  Var_avg = readInputRegisterValues(F7M24, 0x224, 2);

  // Apparent Power Total - St (VA)
  Va_avg = readInputRegisterValues(F7M24, 0x22C, 2);
  delay(100);
}

/**
  Requests and retrieves maximum electrical,and power information from Energy meter over Modbus RTU protocol.
*/
void modbus_com_max(){
  // Maximum Measurements
  // Voltage (V)
  V_max = readInputRegisterValues(F7M24, 0x25F, 2);

  // Current (A)
  A_max = readInputRegisterValues(F7M24, 0x272, 2);

  // Active Power Total - Pt (W)
  W_max = readInputRegisterValues(F7M24, 0x280, 2);
  
  // Reactive Power Total - Qt (var)
  Var_max = readInputRegisterValues(F7M24, 0x288, 2);

  // Apparent Power Total - St (VA)
  Va_max = readInputRegisterValues(F7M24, 0x290, 2);
  delay(100);
}

/**
  Requests and retrieves minimum electrical, and power information from Energy meter over Modbus RTU protocol.
*/
void modbus_com_min(){
  // Minimum Measurements
  // Voltage (V)
  V_min = readInputRegisterValues(F7M24, 0x2C3, 2);

  // Current (A)
  A_min = readInputRegisterValues(F7M24, 0x2D6, 2);

  // Active Power Total - Pt (W)
  W_min = readInputRegisterValues(F7M24, 0x2E4, 2);
  
  // Reactive Power Total - Qt (var)
  Var_min = readInputRegisterValues(F7M24, 0x2EC, 2);

  // Apparent Power Total - St (VA)
  Va_min = readInputRegisterValues(F7M24, 0x2F4, 2);
  delay(100);
}

/**
  Requests and retrieves energy information from Energy meter over Modbus RTU protocol.
*/
void modbus_com_energy(){
  // Energy
  // Energy (Wh) - n1
  Wh_packet = readInputRegisterValues(F7M24, 0xAC0, 2);

  // Energy (varh) - n2
  Varh_packet = readInputRegisterValues(F7M24, 0x2F2, 2);
  delay(100);
}

/*************************************
* LED PLC Switches
*************************************/

/**
  Defaulting LED states to low when called. 
*/
void plc_led_off() {
  digitalWrite(LEDG, LOW);
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDB, LOW);
  digitalWrite(PIN_SPI_MISO, LOW);
  digitalWrite(PIN_SPI_MOSI, LOW);
  digitalWrite(PIN_SPI_SCK, LOW);
  digitalWrite(PIN_SPI_SS, LOW);
}

/**
  Sets up the LEDs as outputs and calls the plc_led_off() function.
*/
void plc_led_Setup(){
  pinMode(LEDG, OUTPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDB, OUTPUT);
  pinMode(PIN_SPI_MISO, OUTPUT);
  pinMode(PIN_SPI_MOSI, OUTPUT);
  pinMode(PIN_SPI_SCK, OUTPUT);
  pinMode(PIN_SPI_SS, OUTPUT);

  plc_led_off();
}

/*************************************
* Digital Port related tasks
*************************************/

/**
  Sets up the desired output relay to control the pump.
*/
void digitalIO_Setup(){
  pinMode(D0, OUTPUT);
  pinMode(D1, OUTPUT);
}

/*************************************
* Analog Port related tasks
*************************************/

/**
  Sets up analog ports with 12 bit resolution.
*/
void analogIO_Setup(){
  analogReadResolution(12);

  start_time = millis();
  SCB_DisableDCache();
  Serial.println("Start");
}

/*************************************
* RTU related tasks
*************************************/

/**
  Sets up Modbus RTU protocol configuration.
*/
void RTU_Setup(){
  Serial.println("Energy Management - Modbus RTU Client");

  RS485.setDelays(preDelayBR, postDelayBR);

  // start the Modbus RTU client
  // 7M.24 Energy meter specifies 19200 of default baudrate and 8N2 frame
  if (!ModbusRTUClient.begin(baudrate, SERIAL_8N2)) {
    Serial.println("Failed to start Modbus RTU Client!");
    while (1)
        ;
  }
}

/**
  Writes Coil values given argument inputs. 

  @param dev_address Device address.
  @param reg_address Register address.
  @param coil_write Data to write.
  @param byte_count Number of bytes.
*/
void writeCoilValues(int dev_address, uint8_t reg_address, uint8_t coil_write, int byte_count){
  ModbusRTUClient.beginTransmission(dev_address, COILS, reg_address, byte_count);
  ModbusRTUClient.write(coil_write);
  
  if (!ModbusRTUClient.endTransmission()) {
    Serial.print("failed! ");
    Serial.println(ModbusRTUClient.lastError());
  } else {
    Serial.println("success");
  }
}

/**
  Reads Coil values given argument inputs. 

  @param dev_address Device address.
  @param reg_address Register address.
  @param byte_count Number of bytes.
  @param packet Holding register value reading.
*/
void readCoilValues(int dev_address, uint8_t reg_address, int byte_count, int32_t packet){
  Serial.print("Reading Coil values ... ");

  // read 10 Coil values from (slave) id 42, address 0x00
  if (!ModbusRTUClient.requestFrom(dev_address, COILS, reg_address, byte_count)) {
    Serial.print("failed! ");
    Serial.println(ModbusRTUClient.lastError());
  } else {
    Serial.println("success");

    while (ModbusRTUClient.available()) {
        Serial.print(ModbusRTUClient.read());
        packet = ModbusRTUClient.read();
        Serial.print(' ');
    }
    
    Serial.println();
  }
}

/**
  Reads Discrete Input Register values given argument inputs. 

  @param dev_address Device address.
  @param reg_address Register address.
  @param byte_count Number of bytes.
  @param packet Holding register value reading.
*/
void readDiscreteInputValues(int dev_address, uint8_t reg_address, int byte_count, int32_t packet){
  if (!ModbusRTUClient.requestFrom(dev_address, DISCRETE_INPUTS, reg_address, byte_count)) {
    Serial.print("failed! ");
    Serial.println(ModbusRTUClient.lastError());
  } else {
    Serial.println("success");

    while (ModbusRTUClient.available()) {
        Serial.print(ModbusRTUClient.read());
        packet = ModbusRTUClient.read();
        Serial.print(' ');
    }
    Serial.println();
  }
}

/**
  Writes Holding Register values given argument inputs. 

  @param dev_address Device address.
  @param reg_address Register address.
  @param holding_write Data to write.
  @param byte_count Number of bytes.
*/
void writeHoldingRegisterValues(int dev_address, uint8_t reg_address, uint8_t holding_write, int byte_count){
  ModbusRTUClient.beginTransmission(dev_address, HOLDING_REGISTERS, reg_address, byte_count);
  ModbusRTUClient.write(holding_write);

  if (!ModbusRTUClient.endTransmission()) {
    Serial.print("failed! ");
    Serial.println(ModbusRTUClient.lastError());
  } else {
    Serial.println("success");
  }
}

/**
  Reads Holding Register values given argument inputs. 

  @param dev_address Device address.
  @param reg_address Register address.
  @param byte_count Number of bytes.
  @param packet Holding register value reading.
*/
void readHoldingRegisterValues(int dev_address, uint8_t reg_address, int byte_count, int32_t packet){
  if (!ModbusRTUClient.requestFrom(dev_address, HOLDING_REGISTERS, reg_address, byte_count)) {
    Serial.print("failed! ");
    Serial.println(ModbusRTUClient.lastError());
  } else {
    Serial.println("success");

    while (ModbusRTUClient.available()) {
        Serial.print(ModbusRTUClient.read());
        packet = ModbusRTUClient.read();
        Serial.print(' ');
    }
    Serial.println();
  }
}

/**
  Reads Input Register values given argument inputs. 

  @param dev_address Device address.
  @param reg_address Register address.
  @param byte_count Number of bytes.
*/
uint8_t readInputRegisterValues(int dev_address, uint8_t reg_address, int byte_count){
  uint8_t packet;
  if (!ModbusRTUClient.requestFrom(dev_address, INPUT_REGISTERS, reg_address, byte_count)) {
    Serial.print("failed! ");
    Serial.println(ModbusRTUClient.lastError());
    return 0;
  } else {
    Serial.println("success");

    while (ModbusRTUClient.available()) {
        packet = ModbusRTUClient.read();
        Serial.println(packet);
    }
    return packet;
  }
}

/*****************************
* Arduino Cloud 
*****************************/

/**
  Sets up configuration for Arduino Cloud
*/
void iot_cloud_setup(){
  // Defined in thingProperties.h
  initProperties();

  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  
  /*
     The following function allows you to obtain more information
     related to the state of network and IoT Cloud connection and errors
     the higher number the more granular information you’ll get.
     The default is 0 (only errors).
     Maximum is 4
 */
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
  
  // Configure values at which the limit the user desires to operate within and share with Arduino Cloud
  consumption_profile(1.1, 120, 2880);
}

/*
  Since UOperMargin is READ_WRITE variable, onUOperMarginChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onUOperMarginChange()  {
  Serial.println(F("Energy Manager: New operation margin input value acquired - Arduino IoT Cloud Trigger"));
  if (uOperMargin > 1){
    user_profile.uV_code = uOperMargin;
  }
}
/*
  Since UWatt is READ_WRITE variable, onUWattChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onUWattChange()  {
  Serial.println(F("Energy Manager: New power input value acquired - Arduino IoT Cloud Trigger"));
  if (uWatt > 1){
    user_profile.uW_code = uWatt;
  }
}

/*
  Since UWhCon is READ_WRITE variable, onUWhConChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onUWhConChange()  {
  Serial.println(F("Energy Manager: New energy [Wh] input value acquired - Arduino IoT Cloud Trigger"));
  if (uWhCon > 1){
    user_profile.uWh_code = uWhCon;
  }
}

/*
  Since DirectOverride1 is READ_WRITE variable, onDirectOverride1Change() is
  executed every time a new value is received from IoT Cloud.
*/
void onDirectOverride1Change()  {
  if (directOverride1 == true){
    Serial.println(F("Device #1 Direct Override Requested - Arduino IoT Cloud Trigger"));
    digitalWrite(D0, HIGH); 
  } 
  if (directOverride1 == false){
    // D0 will operate depending on the system's resource state
    Serial.println(F("Device #1 is now cotrolled by the system - Arduino IoT Cloud Trigger"));
  } 
}

/*****************************
* Main
*****************************/
void setup() {
  // Initial Parameter 
  directOverride1 = false;
  uWhOpta = 0;

  Serial.begin(9600);
  while (!Serial);

  delay(1000);

  // Analog/Digital IO Port Configuration
  analogIO_Setup();
  digitalIO_Setup();

  // Modbus RTU Configuration 
  RTU_Setup();
  
  // Status LED configuration;
  plc_led_Setup();

  // IoT Cloud Setup
  iot_cloud_setup();

  // Only for Device On State flag
  digitalWrite(LEDG, HIGH);

  // Scheduler -> ModBus
  Scheduler.startLoop(modbus_line);
}

void loop() {
  // Cloud exchange
  consumption_var_container();
  ArduinoCloud.update();

  // Profile based energy management tasks
  energy_distro_ctrl();

  delay(1000);
}

/**
  Dedicated function for scheduler to retrieve Energy meter's information over Modbus RTU protocol
*/
void modbus_line(){
  modbus_com_actual();
  modbus_com_avg();
  modbus_com_max();
  modbus_com_min();
  modbus_com_energy();
  modbus_com_monitor();
}
