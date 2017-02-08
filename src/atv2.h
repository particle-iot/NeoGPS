#ifndef ATV2_H
#define ATV2_H

#include "Particle.h"
#include <NMEAGPS.h>
#include "math.h"


class AssetTracker {

 public:

  AssetTracker();

  void
    begin(void),
    updateGPS(void),
    gpsOn(void),
    gpsOff(void);
  int
    readX(void),
    readY(void),
    readZ(void),
    readXYZmagnitude(void);
  float
    readLat(void),
    readLon(void),
    readLatDeg(void),
    readLonDeg(void),
    readHDOP(void),
    getGpsAccuracy(void);

  bool
    gpsFix(void);
  char
    checkGPS(void),
    *preNMEA(void);
  String
    readLatLon(void);

  bool
    setupLowPowerWakeMode(uint8_t movementThreshold = 16);
  uint8_t
    clearAccelInterrupt();
  uint32_t 
    getGpsTimestamp();

 private:

};

/*
* Copyright (C) 2008 The Android Open Source Project
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software< /span>
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

/* Update by K. Townsend (Adafruit Industries) for lighter typedefs, and
 * extended sensor support to include color, voltage and current */


/* Intentionally modeled after sensors.h in the Android API:
 * https://github.com/android/platform_hardware_libhardware/blob/master/include/hardware/sensors.h */

/* Constants */
#define SENSORS_GRAVITY_EARTH             (9.80665F)              /**< Earth's gravity in m/s^2 */
#define SENSORS_GRAVITY_MOON              (1.6F)                  /**< The moon's gravity in m/s^2 */
#define SENSORS_GRAVITY_SUN               (275.0F)                /**< The sun's gravity in m/s^2 */
#define SENSORS_GRAVITY_STANDARD          (SENSORS_GRAVITY_EARTH)
#define SENSORS_MAGFIELD_EARTH_MAX        (60.0F)                 /**< Maximum magnetic field on Earth's surface */
#define SENSORS_MAGFIELD_EARTH_MIN        (30.0F)                 /**< Minimum magnetic field on Earth's surface */
#define SENSORS_PRESSURE_SEALEVELHPA      (1013.25F)              /**< Average sea level pressure is 1013.25 hPa */
#define SENSORS_DPS_TO_RADS               (0.017453293F)          /**< Degrees/s to rad/s multiplier */
#define SENSORS_GAUSS_TO_MICROTESLA       (100)                   /**< Gauss to micro-Tesla multiplier */

/** Sensor types */
typedef enum
{
  SENSOR_TYPE_ACCELEROMETER         = (1),   /**< Gravity + linear acceleration */
  SENSOR_TYPE_MAGNETIC_FIELD        = (2),
  SENSOR_TYPE_ORIENTATION           = (3),
  SENSOR_TYPE_GYROSCOPE             = (4),
  SENSOR_TYPE_LIGHT                 = (5),
  SENSOR_TYPE_PRESSURE              = (6),
  SENSOR_TYPE_PROXIMITY             = (8),
  SENSOR_TYPE_GRAVITY               = (9),
  SENSOR_TYPE_LINEAR_ACCELERATION   = (10),  /**< Acceleration not including gravity */
  SENSOR_TYPE_ROTATION_VECTOR       = (11),
  SENSOR_TYPE_RELATIVE_HUMIDITY     = (12),
  SENSOR_TYPE_AMBIENT_TEMPERATURE   = (13),
  SENSOR_TYPE_VOLTAGE               = (15),
  SENSOR_TYPE_CURRENT               = (16),
  SENSOR_TYPE_COLOR                 = (17)
} sensors_type_t;

/** struct sensors_vec_s is used to return a vector in a common format. */
typedef struct {
    union {
        float v[3];
        struct {
            float x;
            float y;
            float z;
        };
        /* Orientation sensors */
        struct {
            float roll;    /**< Rotation around the longitudinal axis (the plane body, 'X axis'). Roll is positive and increasing when moving downward. -90°<=roll<=90° */
            float pitch;   /**< Rotation around the lateral axis (the wing span, 'Y axis'). Pitch is positive and increasing when moving upwards. -180°<=pitch<=180°) */
            float heading; /**< Angle between the longitudinal axis (the plane body) and magnetic north, measured clockwise when viewing from the top of the device. 0-359° */
        };
    };
    uint8_t status;
    uint8_t reserved[3];
} sensors_vec_t;

/** struct sensors_color_s is used to return color data in a common format. */
typedef struct {
    union {
        float c[3];
        /* RGB color space */
        struct {
            float r;       /**< Red component */
            float g;       /**< Green component */
            float b;       /**< Blue component */
        };
    };
    uint32_t rgba;         /**< 24-bit RGBA value */
} sensors_color_t;

/* Sensor event (36 bytes) */
/** struct sensor_event_s is used to provide a single sensor event in a common format. */
typedef struct
{
    int32_t version;                          /**< must be sizeof(struct sensors_event_t) */
    int32_t sensor_id;                        /**< unique sensor identifier */
    int32_t type;                             /**< sensor type */
    int32_t reserved0;                        /**< reserved */
    int32_t timestamp;                        /**< time is in milliseconds */
    union
    {
        float           data[4];
        sensors_vec_t   acceleration;         /**< acceleration values are in meter per second per second (m/s^2) */
        sensors_vec_t   magnetic;             /**< magnetic vector values are in micro-Tesla (uT) */
        sensors_vec_t   orientation;          /**< orientation values are in degrees */
        sensors_vec_t   gyro;                 /**< gyroscope values are in rad/s */
        float           temperature;          /**< temperature is in degrees centigrade (Celsius) */
        float           distance;             /**< distance in centimeters */
        float           light;                /**< light in SI lux units */
        float           pressure;             /**< pressure in hectopascal (hPa) */
        float           relative_humidity;    /**< relative humidity in percent */
        float           current;              /**< current in milliamps (mA) */
        float           voltage;              /**< voltage in volts (V) */
        sensors_color_t color;                /**< color in RGB component values */
    };
} sensors_event_t;

/* Sensor details (40 bytes) */
/** struct sensor_s is used to describe basic information about a specific sensor. */
typedef struct
{
    char     name[12];                        /**< sensor name */
    int32_t  version;                         /**< version of the hardware + driver */
    int32_t  sensor_id;                       /**< unique sensor identifier */
    int32_t  type;                            /**< this sensor's type (ex. SENSOR_TYPE_LIGHT) */
    float    max_value;                       /**< maximum value of this sensor's value in SI units */
    float    min_value;                       /**< minimum value of this sensor's value in SI units */
    float    resolution;                      /**< smallest difference between two values reported by this sensor */
    int32_t  min_delay;                       /**< min delay in microseconds between events. zero = not a constant rate */
} sensor_t;

class Adafruit_Sensor {
 public:
  // Constructor(s)
  Adafruit_Sensor() {}
  virtual ~Adafruit_Sensor() {}

  // These must be defined by the subclass
  virtual void enableAutoRange(bool enabled) {};
  virtual bool getEvent(sensors_event_t*) = 0;
  virtual void getSensor(sensor_t*) = 0;
  
 private:
  bool _autoRange;
};


/**************************************************************************/
/*!
    @file     Adafruit_LIS3DH.h
    @author   K. Townsend / Limor Fried (Adafruit Industries)
    @license  BSD (see license.txt)

    This is a library for the Adafruit LIS3DH Accel breakout board
    ----> https://www.adafruit.com/products/????

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

 //#include "application.h"


//#include <Wire.h>
//#include <SPI.h>
//#include "Adafruit_Sensor.h"

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define LIS3DH_DEFAULT_ADDRESS  (0x18)    // if SDO/SA0 is 3V, its 0x19
/*=========================================================================*/

#define LIS3DH_REG_STATUS1       0x07
#define LIS3DH_REG_OUTADC1_L     0x08
#define LIS3DH_REG_OUTADC1_H     0x09
#define LIS3DH_REG_OUTADC2_L     0x0A
#define LIS3DH_REG_OUTADC2_H     0x0B
#define LIS3DH_REG_OUTADC3_L     0x0C
#define LIS3DH_REG_OUTADC3_H     0x0D
#define LIS3DH_REG_INTCOUNT      0x0E
#define LIS3DH_REG_WHOAMI        0x0F
#define LIS3DH_REG_TEMPCFG       0x1F
#define LIS3DH_REG_CTRL1         0x20
#define LIS3DH_REG_CTRL2         0x21
#define LIS3DH_REG_CTRL3         0x22
#define LIS3DH_REG_CTRL4         0x23
#define LIS3DH_REG_CTRL5         0x24
#define LIS3DH_REG_CTRL6         0x25
#define LIS3DH_REG_REFERENCE     0x26
#define LIS3DH_REG_STATUS2       0x27
#define LIS3DH_REG_OUT_X_L       0x28
#define LIS3DH_REG_OUT_X_H       0x29
#define LIS3DH_REG_OUT_Y_L       0x2A
#define LIS3DH_REG_OUT_Y_H       0x2B
#define LIS3DH_REG_OUT_Z_L       0x2C
#define LIS3DH_REG_OUT_Z_H       0x2D
#define LIS3DH_REG_FIFOCTRL      0x2E
#define LIS3DH_REG_FIFOSRC       0x2F
#define LIS3DH_REG_INT1CFG       0x30
#define LIS3DH_REG_INT1SRC       0x31
#define LIS3DH_REG_INT1THS       0x32
#define LIS3DH_REG_INT1DUR       0x33
#define LIS3DH_REG_CLICKCFG      0x38
#define LIS3DH_REG_CLICKSRC      0x39
#define LIS3DH_REG_CLICKTHS      0x3A
#define LIS3DH_REG_TIMELIMIT     0x3B
#define LIS3DH_REG_TIMELATENCY   0x3C
#define LIS3DH_REG_TIMEWINDOW    0x3D

typedef enum
{
  LIS3DH_RANGE_16_G         = 0b11,   // +/- 16g
  LIS3DH_RANGE_8_G           = 0b10,   // +/- 8g
  LIS3DH_RANGE_4_G           = 0b01,   // +/- 4g
  LIS3DH_RANGE_2_G           = 0b00    // +/- 2g (default value)
} lis3dh_range_t;


/* Used with register 0x2A (LIS3DH_REG_CTRL_REG1) to set bandwidth */
typedef enum
{
  LIS3DH_DATARATE_400_HZ     = 0b0111, //  400Hz 
  LIS3DH_DATARATE_200_HZ     = 0b0110, //  200Hz
  LIS3DH_DATARATE_100_HZ     = 0b0101, //  100Hz
  LIS3DH_DATARATE_50_HZ      = 0b0100, //   50Hz
  LIS3DH_DATARATE_25_HZ      = 0b0011, //   25Hz
  LIS3DH_DATARATE_10_HZ      = 0b0010, // 10 Hz
  LIS3DH_DATARATE_1_HZ       = 0b0001, // 1 Hz
  LIS3DH_DATARATE_POWERDOWN  = 0,
  LIS3DH_DATARATE_LOWPOWER_1K6HZ  = 0b1000,
  LIS3DH_DATARATE_LOWPOWER_5KHZ  =  0b1001,

} lis3dh_dataRate_t;

typedef enum {
  LIS3DH_CTRL_REG1_ODR3 = 0x80,
  LIS3DH_CTRL_REG1_ODR2 = 0x40,
  LIS3DH_CTRL_REG1_ODR1 = 0x20,
  LIS3DH_CTRL_REG1_ODR0 = 0x10,
  LIS3DH_CTRL_REG1_LPEN = 0x08,
  LIS3DH_CTRL_REG1_ZEN = 0x04,
  LIS3DH_CTRL_REG1_YEN = 0x02,
  LIS3DH_CTRL_REG1_XEN = 0x01
} list3dh_ctrl_reg1_t;

typedef enum {
  LIS3DH_CTRL_REG2_HPM1 = 0x80,
  LIS3DH_CTRL_REG2_HPM0 = 0x40,
  LIS3DH_CTRL_REG2_HPCF2 = 0x20,
  LIS3DH_CTRL_REG2_HPCF1 = 0x10,
  LIS3DH_CTRL_REG2_FDS = 0x08,
  LIS3DH_CTRL_REG2_HPCLICK = 0x04,
  LIS3DH_CTRL_REG2_HPIS2 = 0x02,
  LIS3DH_CTRL_REG2_HPIS1 = 0x01
} list3dh_ctrl_reg2_t;


typedef enum {
  LIS3DH_CTRL_REG3_I1_CLICK = 0x80,
  LIS3DH_CTRL_REG3_I1_INT1 = 0x40,
  LIS3DH_CTRL_REG3_I1_DRDY = 0x10,
  LIS3DH_CTRL_REG3_I1_WTM = 0x04,
  LIS3DH_CTRL_REG3_I1_OVERRUN = 0x02
} list3dh_ctrl_reg3_t;

typedef enum {
  LIS3DH_CTRL_REG4_BDU = 0x80,
  LIS3DH_CTRL_REG4_BLE = 0x40,
  LIS3DH_CTRL_REG4_FS1 = 0x20,
  LIS3DH_CTRL_REG4_FS0 = 0x10,
  LIS3DH_CTRL_REG4_HR = 0x08,
  LIS3DH_CTRL_REG4_ST1 = 0x04,
  LIS3DH_CTRL_REG4_ST0 = 0x02,
  LIS3DH_CTRL_REG4_SIM = 0x01
} list3dh_ctrl_reg4_t;


typedef enum {
  LIS3DH_CTRL_REG5_BOOT = 0x80,
  LIS3DH_CTRL_REG5_FIFO_EN = 0x40,
  LIS3DH_CTRL_REG5_LIR_INT1 = 0x08,
  LIS3DH_CTRL_REG5_D4D_INT1 = 0x04
} list3dh_ctrl_reg5_t;

typedef enum {
  LIS3DH_CTRL_REG6_I2_CLICK = 0x80,
  LIS3DH_CTRL_REG6_I2_INT2 = 0x40,
  LIS3DH_CTRL_REG6_BOOT_I2 = 0x10,
  LIS3DH_CTRL_REG6_H_LACTIVE = 0x02
} list3dh_ctrl_reg6_t;

typedef enum {
  LIS3DH_INT1_CFG_AOI = 0x80,
  LIS3DH_INT1_CFG_6D = 0x40,
  LIS3DH_INT1_CFG_ZHIE_ZUPE = 0x20,
  LIS3DH_INT1_CFG_ZLIE_ZDOWNE = 0x10,
  LIS3DH_INT1_CFG_YHIE_YUPE = 0x08,
  LIS3DH_INT1_CFG_YLIE_YDOWNE = 0x04,
  LIS3DH_INT1_CFG_XHIE_XUPE = 0x02,
  LIS3DH_INT1_CFG_XLIE_XDOWNE = 0x01
} list3dh_ctrl_int1_cfg_t;

typedef enum {
  LIS3DH_INT1_SRC_IA = 0x40,
  LIS3DH_INT1_SRC_ZH = 0x20,
  LIS3DH_INT1_SRC_ZL = 0x10,
  LIS3DH_INT1_SRC_YH = 0x08,
  LIS3DH_INT1_SRC_YL = 0x04,
  LIS3DH_INT1_SRC_XH = 0x02,
  LIS3DH_INT1_SRC_XL = 0x01
} list3dh_ctrl_int2_src_t;

class Adafruit_LIS3DH : public Adafruit_Sensor {
 public:
  Adafruit_LIS3DH(void);
  Adafruit_LIS3DH(int8_t cspin);
  Adafruit_LIS3DH(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin);
  
  bool       begin(uint8_t addr = LIS3DH_DEFAULT_ADDRESS);

  void read();
  uint16_t readADC(uint8_t a);

  void setRange(lis3dh_range_t range);
  lis3dh_range_t getRange(void);

  void setDataRate(lis3dh_dataRate_t dataRate);
  lis3dh_dataRate_t getDataRate(void);

  bool getEvent(sensors_event_t *event);
  void getSensor(sensor_t *sensor);

  uint8_t getOrientation(void);

  bool setupLowPowerWakeMode(uint8_t movementThreshold);
  uint8_t clearInterrupt();

  int16_t x, y, z;
  float x_g, y_g, z_g;

  
 private:
  uint8_t readRegister8(uint8_t reg);
  void writeRegister8(uint8_t reg, uint8_t value);
  uint8_t spixfer(uint8_t x = 0xFF);
  void beginTransaction();
  void endTransaction();

  int32_t _sensorID;
  int8_t  _i2caddr;

  // SPI
  int8_t _cs, _mosi, _miso, _sck;

  // LIS3DH INT1 is connected to this pin
  int intPin = WKP;
  uint8_t int1_cfg;
};



#endif