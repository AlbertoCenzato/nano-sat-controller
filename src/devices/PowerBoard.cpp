#include "devices/PowerBoard.hpp"

#include "utils/Settings.hpp"

/* PowerBoard.Python interface to the MoPi battery power add - on board for the

#   Raspberry Pi. (http://pi.gate.ac.uk/mopi)

# TODO
# 
# bit 15: was low; now high : use it to determine type of board, and add a way
# to query that
#
# add logic about firmware version : for the old board, expect <= 3.5; for the
# new one, >= 4.0; change the version error to a warning
#
# add 2 bytes to the board number read : "MoPi-2 is using a unique 32-bit laser
# burned code into the microcontroller as serial number.The syntax of that
# command is the same, please only extend it to read 4, instead of 2 bytes
# from MoPi."
#
# test the charging controller :
# - For proper functioning it is assumed that both, Input #1 and external
#   charger, are powered by one common source.Handling of different sources
#   is rising the hardware complexity.
# - In fact no charging functionality. Responsibility of the process is
#   transferred to the external charger connected to the 2 - pin screw terminal.
#   MoPi - 2 only enables this input(green LED on) when there is an input power
#   at input #1, a battery pack is connected to the input #2 and charger
#   voltage is applied.Then the firmware disables the charging gate in case
#   the input #1 voltage disappears(switching RPi powering to the battery on
   #   input #2) or charging voltage disappears.Time of charging, algorithm,
   #   limits etc.are functions of the external charger.
# - Charging starts only after the power button is pressed. 
# - Charging stops when MoPi goes off. If however MoPi is in idle mode,
   #   counting power up delay, the charging continuous.Not sure the last is
   #   working in the prototypes, but if not, will be fixed soon.
#
   # investigate possibility of propagating power reading :
# - There is a possibility to get info about the applied load (current
#   consumption).The firmware can measure the voltage drop on the splitting
#   MoPi - 2 and RasPi MOSFET transistor.The measurement however will be rough
#   as the transistor's resistance in saturation differs from item to item.
#   But we can put a limit, e.g.around 2.5A load, and light / flash a special
#   red LED.The accuracy will be around 15 % .Do you think this will be useful
#   for the customer ?
# - Unfortunately we do not have spare bits in the status word right now to
#   propagate this signal to the driver.Is it difficult to add one additional
#   byte or word to MoPi status ? In that case we will be able to give a little
#   more info to the user, e.g. 2 - bit field giving 4 ranges of load.The LED
#   we can keep only one signaling the high - current load state.
*/


// Version of the API
#define APIVERSION 0.3f

// For at least mopi firmware vX.YY...
#define FIRMMAJ  3
#define FIRMMINR 5

// Package version
#define VERSION "4.1"

// Number of times to retry a failed I2C read / write to the MoPi
#define MAXTRIES 3

namespace sat {
namespace device {

status::status(uint16_t status) : byte(status) { }

uint16_t status::getByte() const { return byte; }

// get the bit, starting from 0 LSB
int status::getBit(int bitnum) const {
   return (byte & (1 << bitnum)) >> bitnum;
}

int status::SourceOneActive() const { return getBit(0); }
int status::SourceTwoActive() const { return getBit(1); }
int status::LEDBlue()  const { return getBit(2); }
int status::LEDGreen() const { return getBit(3); }
int status::LEDRed()   const { return getBit(4); }
int status::LEDFlashing() const { return getBit(5); }

// Output: 1 for NiMH, 0 for Alkaline
int status::JumperState() const { return !getBit(6); }
int status::ForcedShutdown() const { return  getBit(7); }

int status::PowerOnDelaySet()     const { return getBit(8); }
int status::PowerOnDelayActive()  const { return getBit(9); }
int status::ShutdownDelaySet()    const { return getBit(10); }
int status::ShutdownDelayActive() const { return getBit(11); }
int status::CheckSourceOne()      const { return getBit(12); }
int status::CheckSourceTwo()      const { return getBit(13); }
int status::UserConfiguration()   const { return getBit(14); }

std::string status::StatusDetail() const {

   std::string out;

   if (SourceOneActive())
      out += "Source #1 active\n";

   if (SourceTwoActive())
      out += "Source #2 active\n";

   if (LEDBlue())
      out += "Source full (blue led)\n";

   if (LEDGreen())
      out += "Source good (green led)\n";

   if (LEDRed())
      out += "Source low (red led)\n";

   if (LEDFlashing())
      out += "Source critical (flashing red led)\n";

   if (!UserConfiguration()) {
      if (JumperState())
         out += "NiMH battery profile\n";
      else
         out += "Alkaline battery profile\n";
   }

   if (ForcedShutdown())
      out += "Forced shutdown\n";

   if (PowerOnDelaySet())
      out += "Power on delay set\n";

   if (PowerOnDelayActive())
      out += "Power on delay in progress\n";

   if (ShutdownDelaySet())
      out += "Shutdown delay set\n";

   if (ShutdownDelayActive())
      out += "Shutdown delay in progress\n";

   if (CheckSourceOne())
      out += "Source #1 good\n";
   else
      out += "Source #1 low/not present\n";

   if (CheckSourceTwo())
      out += "Source #2 good\n";
   else
      out += "Source #2 low/not present\n";

   if (UserConfiguration())
      out += "User configured\n";

   return out;
}



   const string PowerBoard::DEFAULT_DEV_NAME = "Mopi_power_board";


PowerBoard::PowerBoard(gnublin_i2c *bus, const utils::PowerBoardSettings& settings)   
   : DeviceI2C(settings.deviceID, bus, settings.address) {

   std::tie(maj, minr) = getFirmwareVersion();

   if (maj != FIRMMAJ || minr < FIRMMINR) {
      stringstream errorMsg;
      errorMsg << "Expected at least MoPi firmware version " << FIRMMAJ << "." << FIRMMINR 
               << ", got " << maj << "." << minr << " instead.";
      Log::err  << errorMsg.str();
      std::cerr << errorMsg.str() << std::endl;
   }
}

std::vector<double> PowerBoard::read() const {
   return { double(getVoltage()) };
}

status PowerBoard::getStatus() const {
   uint16_t value;
   read16(0b00000000, value);
   if (maj == 3 && minr > 9) {
      // bit changed at v3.10
      value = value ^ (1 << 6);
   }
   return {value};
}

uint16_t PowerBoard::getVoltage(int input) const {
   uint16_t value;
   if (input == 1)
      read16(0b00000101, value); // 5
   else if (input == 2)
      read16(0b00000110, value); // 6
   else
      read16(0b00000001, value);
   return value;
}

// returns an array of 5 integers: power source type, max, good, low, crit(mV)
std::array<int,5> PowerBoard::readConfig(int input) const {
   
   //try reading the config
   int tries = 0;
   uint8_t buffer[5];
   while (tries < MAXTRIES) {
      auto success = true;
      if (input == 1)
         success = readX(0b00000111, buffer, 5); // 7
      else if (input == 2)
         success = readX(0b00001000, buffer, 5); // 8
      else
         success = readX(0b00000010, buffer, 5); // 2

      if (success)
         break;

      std::this_thread::sleep_for(std::chrono::milliseconds(330));
      tries += 1;
   }
   
   array<int, 5> data{ buffer[0], buffer[1], buffer[2], buffer[3], buffer[4] };

   // behaivour changed at v3.10 to 5x0 so that 255 could serve as error detection
   if (maj == 3 && minr > 9 && data == std::array<int,5>{0, 0, 0, 0, 0})
      data = { 255, 255, 255, 255, 255 };

   if (data[0] != 255) {
      // it's a cV reading that we need to convert back to mV
      // (with 255's it's indicating a differing config)
      for (auto i = 1; i < 6; ++i)
         data[i] *= 100;
   }

   return data;
}

// takes an array of 5 integers: power source type, max, good, low, crit(mV)
bool PowerBoard::writeConfig(std::array<int, 5> battery, int input) {

   if (battery[0] < 1 || battery[0] > 3) {
      Log::err << "PowerBoard::writeConfig(): Invalid parameter, type outside range";
      return false;
   }

   uint8_t data[5];
   data[0] = uint8_t(battery[0]);
   for (auto i = 1; i < 6; ++i) {
      battery[i] /= 100;
      if (battery[i] < 0 || battery[i] > 255) {
         Log::err << "PowerBoard::writeConfig(): Invalid parameter, voltage outside range";
         return false;
      }
      data[i] = uint8_t(battery[i]);
      battery[i] *= 100; // for the read back we need to compare to the rounded value
   }

   // check if config to be written matches existing config
   if (battery == readConfig(input))
      return true;

   // try writing the config
   int tries = 0;
   while (tries < MAXTRIES) {
      if (input == 1)
         writeX(0b00000111, data, 5); // 7
      else if (input == 2)
         writeX(0b00001000, data, 5); // 8
      else
         writeX(0b00000010, data, 5); // 2
      
      // read back test
      std::this_thread::sleep_for(std::chrono::milliseconds(20)); // slight delay to allow write to take effect
      if (battery == readConfig(input))
         break;

      tries += 1;
   }

   if (tries == MAXTRIES) {
      Log::err << "PowerBoard::writeConfig(): Communications protocol error on send config";
      return false;
   }

   return true;
}

void PowerBoard::setPowerOnDelay(uint16_t poweron) {
   write16(0b00000011, poweron);
}

void PowerBoard::setShutdownDelay(uint16_t shutdown) {
   write16(0b00000100, shutdown);
}

uint16_t PowerBoard::getPowerOnDelay() const {  // TODO: check uint16_t
   uint16_t value;
   read16(0b00000011, value);
   return value;
}

uint16_t PowerBoard::getShutdownDelay() const {
   uint16_t value;
   read16(0b00000100, value);
   return value;
}

std::pair<std::int8_t, std::int8_t> PowerBoard::getFirmwareVersion() const {
   uint16_t word;
   read16(0b00001001, word); // 9
   return std::make_pair(std::int8_t(word >> 8), std::int8_t(word & 0xff));
}

uint16_t PowerBoard::getSerialNumber() const {
   uint16_t serial;
   read16(0b00001010, serial); // 10
   return serial;
}

float PowerBoard::getApiVersion() const {
   return APIVERSION;
}

TestResult PowerBoard::testConnection() {
   auto result = sat::ConnectionSelfTest::create(deviceID);
   auto serial = getSerialNumber();
   if (serial == 0) {
      result->errorLevel      = ErrorLevel::warning;
      result->additionalInfo += " | invalid serial number";
   }

   return TestResult(result);
}

std::unique_ptr<PowerBoard> PowerBoard::create(gnublin_i2c * bus, const utils::PowerBoardSettings & settings) {
   return std::make_unique<PowerBoard>(bus, settings);
}

}  // namespace device
}  // namespace sat
