#pragma once

#include "devices/DeviceI2C.hpp"

namespace sat {

namespace utils { struct PowerBoardSettings; }

namespace device {


class status {

   uint16_t byte = 0;

public:

   status(uint16_t status);

   uint16_t getByte() const;

   // get the bit, starting from 0 LSB
   int getBit(int bitnum) const;

   int SourceOneActive() const;
   int SourceTwoActive() const;
   int LEDBlue()  const;
   int LEDGreen() const;
   int LEDRed()   const;
   int LEDFlashing() const;

   // Output: 1 for NiMH, 0 for Alkaline
   int JumperState() const;
   int ForcedShutdown() const;

   int PowerOnDelaySet()     const;
   int PowerOnDelayActive()  const;
   int ShutdownDelaySet()    const;
   int ShutdownDelayActive() const;
   int CheckSourceOne()      const;
   int CheckSourceTwo()      const;
   int UserConfiguration()   const;

   std::string StatusDetail() const;

};



class PowerBoard : public DeviceI2C, public ISensor {

private:
    static const std::string DEFAULT_DEV_NAME;
   static const uint8_t DEFAULT_I2C_ADDR = 0x0B;
   std::int8_t maj = 0;
   std::int8_t minr = 0;

public:

   PowerBoard(gnublin_i2c *bus, const utils::PowerBoardSettings& settings);

   std::vector<double> read() const override;

   status getStatus() const;


   uint16_t getVoltage(int input = 0) const;


   // returns an array of 5 integers: power source type, max, good, low, crit(mV)
   std::array<int, 5> readConfig(int input = 0) const;


   // takes an array of 5 integers: power source type, max, good, low, crit(mV)
   bool writeConfig(std::array<int, 5> battery, int input = 0);


   void setPowerOnDelay(uint16_t poweron);


   void setShutdownDelay(uint16_t shutdown);


   uint16_t getPowerOnDelay() const;


   uint16_t getShutdownDelay() const;


   std::pair<std::int8_t, std::int8_t> getFirmwareVersion() const;

   uint16_t getSerialNumber() const;

   /*
   uint16_t baseReadWord(uint16_t reg);


   uint16_t readWord(uint16_t reg);



   uint16_t advancedReadWord(uint16_t reg);



   void writeWord(uint16_t reg, int data);
   */

   float getApiVersion() const;


   TestResult testConnection() override;

   static std::unique_ptr<PowerBoard> create(gnublin_i2c *bus, const utils::PowerBoardSettings& settings);

};


using PowerBoardPtr = std::unique_ptr<PowerBoard>;

} // namespace device
} // namespace sat