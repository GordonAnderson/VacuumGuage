//
// VacuumGauge.cpp
//
// Vacuum pressure gauge built around a Posifa pressure sensor with an I2C
// interface. The firmware targets a LILYGO T-QT Pro (ESP32-S3) with a
// 128 x 128 GC9A01 TFT, driven through TFT_eSPI.
//
// Responsibilities:
//   - Periodically read the sensor over I2C and convert the reading to Torr.
//   - Display the pressure on the TFT, auto-ranging between Torr and mTorr.
//   - Apply user-trimmable zero offsets (coarse Torr / fine mTorr) set via the
//     two on-board buttons or serial commands.
//   - Persist the configuration to SPIFFS and reload it on boot.
//   - Expose a serial command interface (see the command table below).
//
// Arduino IDE board settings used for reference (PlatformIO mirrors these via
// the lilygo-t3-s3 board definition):
//   Board:            ESP32S3 Dev Module
//   USB CDC On Boot:  Enabled
//   CPU Frequency:    240MHz (WiFi)
//   Flash Mode:       QIO 80MHz
//   Flash Size:       4MB (32Mb)
//   Partition Scheme: Default 4MB with spiffs (1.2MB APP/1.5MB SPIFFS)
//   Upload Speed:     921600
//   USB Mode:         Hardware CDC and JTAG
//

#include <Arduino.h>
#include <string.h>

#include "FS.h"
#include "SPIFFS.h"

#include <TFT_eSPI.h>   // Graphics and font library, configured for the GC9A01 panel
#include <SPI.h>
#include <Wire.h>

#include <Button.h>
#include <RingBuffer.h>
#include <commandProcessor.h>
#include <charAllocate.h>
#include <debug.h>

#include <arduino-timer.h>

#include "VacuumGauge.h"

// ---------------------------------------------------------------------------
// Configuration constants
// ---------------------------------------------------------------------------

// Format SPIFFS automatically if the partition fails to mount on first boot.
#define FORMAT_SPIFFS_IF_FAILED true

static const char *kSettingsFile = "/Default.dat";

// TFT geometry (the panel is square).
static const uint16_t screenWidth  = 128;
static const uint16_t screenHeight = 128;

// I2C bus pins for the sensor (T-QT Pro Qwiic/STEMMA header) and bus speed.
static const int  kI2C_SDA      = 43;
static const int  kI2C_SCL      = 44;
static const long kI2C_Hz       = 100000;
static const int  kSensorBytes  = 6;     // Bytes returned by each sensor read

// Periodic task intervals (milliseconds).
static const unsigned long kReadIntervalMs = 500;
static const unsigned long kSaveIntervalMs = 10000;

// On-board buttons (GPIO numbers). Each press nudges the active zero offset.
static const uint8_t kButtonA_Pin = 0;
static const uint8_t kButtonB_Pin = 47;

// Pressure threshold (Torr) that selects which offset trim and which display
// units (Torr vs mTorr region) are in effect.
static const float kTorrThreshold = 10.0f;

// Posifa sensor characteristic: counts 50000..65535 are "trend only" data and
// map to 50000..760000 microns, i.e. ~45.7 microns per count above 50000.
static const float kTrendKnee       = 50000.0f;
static const float kMicronsPerCount = 45.7f;

// Absolute pressure floor (Torr). Reported/displayed pressure is clamped here so
// a negative zero-offset or near-vacuum sensor noise can never show as negative.
static const float kMinPressure     = 0.0f;

// Bounds on the user zero-offset trims (symmetric about zero), so repeated
// button presses - or out-of-range serial command values - can't drive the
// calibration into nonsense.
static const int kMaxOffsetTorr     = 700;    // +/- Torr  (coarse trim)
static const int kMaxOffsetmTorr    = 10000;   // +/- mTorr (fine trim)

const char *Version = "Vacuum Gauge, version 1.0 Feb 3, 2024";

// ---------------------------------------------------------------------------
// Persistent data structure
// ---------------------------------------------------------------------------

typedef struct
{
  int16_t       Size;            // sizeof(Data); used to validate a loaded file
  char          Name[20];        // Board name, "Press"
  int8_t        Rev;             // Board revision number
  int           TWIadd;          // Sensor I2C address
  double        calPress;        // Last computed pressure, Torr  (live, not persisted intent)
  int           rawData;         // Last raw pressure counts      (live)
  int           rawTemp;         // Last raw temperature counts   (live)
  int           offsetTorr;      // Coarse zero offset, Torr
  int           offsetmTorr;     // Fine zero offset, mTorr
  int           useCalTable;     // 0 = factory micron formula, 1 = PWL cal table
} Data;

Data data =
{
  sizeof(Data), "Press", 1,
  0x50,
  0, 0, 0,
  0, 0,
  0                              // useCalTable: default to factory formula
};

// Snapshot of the last values written to flash, used to detect changes.
Data lastSaved;

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------

commandProcessor cp;
debug            dbg(&cp);
Button           buttonA(kButtonA_Pin);
Button           buttonB(kButtonB_Pin);

TFT_eSPI tft = TFT_eSPI();             // Pins/driver defined in the User_Setup
auto     timer = timer_create_default();

// True once SPIFFS has mounted successfully (set in setup()).
static bool spiffsReady = false;

// ---------------------------------------------------------------------------
// Serial command handlers
// ---------------------------------------------------------------------------

void saveSettings(void)
{
  if(!cp.checkExpectedArgs(0)) return;
  if(saveData()) cp.sendACK();
  else           cp.sendNAK();
}

void loadSettings(void)
{
  if(!cp.checkExpectedArgs(0)) return;
  if(loadData()) cp.sendACK();
  else           cp.sendNAK();
}

// ---------------------------------------------------------------------------
// Command table
//
// Columns: name, type, expected-arg-count, target pointer, constraint, help.
// A -1 arg-count disables the argument-count check (used for get/set queries).
// ---------------------------------------------------------------------------

Command cmds[] =
{
  {"GVER",     CMDstr,      0,  (void *)Version,             NULL, "Firmware version"},
  {"?NAME",    CMDstr,     -1,  (void *)&data.Name,          NULL, "Device name"},
  {"LOAD",     CMDfunction, 0,  (void *)loadSettings,        NULL, "Load the saved parameters"},
  {"SAVE",     CMDfunction, 0,  (void *)saveSettings,        NULL, "Save parameters to file"},
  {"GPRES",    CMDdouble,   0,  (void *)&data.calPress,      NULL, "Return pressure in Torr"},
  {"GRPRES",   CMDint,      0,  (void *)&data.rawData,       NULL, "Return raw pressure sensor data"},
  {"GRTEMP",   CMDint,      0,  (void *)&data.rawTemp,       NULL, "Return raw temp sensor data"},
  {"?TOFFSET", CMDint,     -1,  (void *)&data.offsetTorr,    NULL, "Set/return Torr offset"},
  {"?MTOFFSET",CMDint,     -1,  (void *)&data.offsetmTorr,   NULL, "Set/return milli-Torr offset"},
  {"?CALMODE", CMDint,     -1,  (void *)&data.useCalTable,   NULL, "Calibration mode: 0=factory formula, 1=PWL table"},
  {NULL}
};
static CommandList cmdList = {cmds, NULL};

// ---------------------------------------------------------------------------
// Change detection
// ---------------------------------------------------------------------------

// Returns true when any user-persistent setting differs from the copy that was
// last written to flash. The live measurement fields (calPress, rawData,
// rawTemp) change on every read, so they are deliberately excluded - only the
// configuration we actually want to persist is compared. (The previous version
// compared the raw struct bytes, which also compared indeterminate padding and
// could trigger spurious saves.)
static bool settingsChanged(void)
{
  return data.offsetTorr  != lastSaved.offsetTorr  ||
         data.offsetmTorr != lastSaved.offsetmTorr ||
         data.useCalTable != lastSaved.useCalTable ||
         data.TWIadd      != lastSaved.TWIadd      ||
         data.Rev         != lastSaved.Rev         ||
         strncmp(data.Name, lastSaved.Name, sizeof(data.Name)) != 0;
}

// Debug command: report the latest raw pressure count.
//
// NOTE: cp.println() has no String overload - passing an Arduino String (e.g.
// "Raw data: " + String(data.rawData)) silently converts to bool and prints
// "TRUE". Use the typed print()/println() overloads instead.
void Debug(void)
{
  cp.print("Raw data: ");
  cp.println(data.rawData);   // println(int)
//  cp.println(settingsChanged() ? "Changed since last save" : "No change");
}

// ---------------------------------------------------------------------------
// Persistence (SPIFFS)
// ---------------------------------------------------------------------------

bool saveData(void)
{
  if(!spiffsReady) return false;

  File file = SPIFFS.open(kSettingsFile, FILE_WRITE);
  if(!file) return false;

  size_t written = file.write((uint8_t *)&data, sizeof(Data));
  file.close();
  return written == sizeof(Data);
}

bool loadData(void)
{
  if(!spiffsReady) return false;

  File file = SPIFFS.open(kSettingsFile, FILE_READ);
  if(!file) return false;

  Data d;
  size_t read = file.read((uint8_t *)&d, sizeof(Data));
  file.close();

  // Only accept the file if it is the expected size/layout.
  if(read == sizeof(Data) && d.Size == data.Size)
  {
    data = d;
    return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
// Display
// ---------------------------------------------------------------------------

void displayPressure(float pressure)
{
  String pStr;

  tft.drawString("Vacuum pressure", 0, 0, 1);
  tft.drawString("GAACE, 2024", 0, 110, 1);

  // Clear the dynamic value/units region so shrinking numbers or a units
  // change (Torr <-> mTorr) cannot leave ghost characters behind.
  tft.fillRect(0, 30, screenWidth, 66, TFT_BLACK);

  if(pressure > 100.0f)
  {
    // Coarse vacuum: whole Torr.
    pStr = String(pressure, 0);
    tft.drawString(pStr.c_str(), 1, 35, 4);
    tft.drawString("Torr", 45, 70, 4);
  }
  else if(pressure >= 1.0f)
  {
    // Medium vacuum: one decimal of Torr.
    pStr = String(pressure, 1);
    tft.drawString(pStr.c_str(), 1, 35, 4);
    tft.drawString("Torr", 45, 70, 4);
  }
  else
  {
    // Fine vacuum: show in mTorr.
    pStr = String(pressure * 1000.0f, 0);
    tft.drawString(pStr.c_str(), 1, 35, 4);
    tft.drawString("mTorr", 45, 70, 4);
  }
}

// ---------------------------------------------------------------------------
// Piece-wise linear (PWL) calibration table
//
// Maps the raw sensor reading (data.rawData) to pressure in Torr by linear
// interpolation between measured calibration points. This is an OPTIONAL
// alternative to the factory micron formula, selected by data.useCalTable
// (see the ?CALMODE command).
//
// Points MUST be sorted by ASCENDING raw value. Note that pressure DECREASES as
// the raw count increases. This is the factory-supplied default table; a future
// user-calibration routine can overwrite these points and persist them.
// ---------------------------------------------------------------------------

typedef struct
{
  int   raw;     // Raw sensor count (as returned in data.rawData / GRPRES)
  float torr;    // Corresponding pressure, Torr
} CalPoint;

static const CalPoint calTable[] =
{
  {18836, 760.00f},
  {21012, 28.90f},
  {21916, 18.70f},
  {24386,  8.57f},
  {25868,  5.94f},
  {27390,  4.14f},
  {28354,  3.39f},
  {29279,  2.77f},
  {30015,  2.38f},
  {30556,  2.08f},
  {30777,  1.97f},
  {31484,  1.70f},
};
static const int calTableSize = sizeof(calTable) / sizeof(calTable[0]);

// Interpolate pressure (Torr) from a raw sensor count using calTable. Readings
// outside the calibrated range are EXTRAPOLATED along the slope of the nearest
// end segment rather than clamped, so the gauge stays responsive just beyond the
// table. (Extrapolation is a straight-line guess; the wider you go, the larger
// the error - add calibration points to extend the trustworthy range. A negative
// extrapolated result is floored later by the kMinPressure clamp.)
static float pwlPressure(int raw)
{
  // Find the segment [i-1, i] to use. Interior readings land in their bracketing
  // segment; readings below the first point reuse the first segment and those
  // above the last point reuse the last segment - both then extrapolate.
  int i = 1;
  while(i < calTableSize - 1 && raw >= calTable[i].raw) i++;

  float r0 = calTable[i - 1].raw,  r1 = calTable[i].raw;
  float p0 = calTable[i - 1].torr, p1 = calTable[i].torr;
  return p0 + (p1 - p0) * ((float)(raw - r0) / (r1 - r0));
}

// ---------------------------------------------------------------------------
// Sensor reading (periodic task)
// ---------------------------------------------------------------------------

// Reads kSensorBytes from the sensor and returns the 16-bit value packed in
// bytes [1] (high) and [2] (low). Returns true if the expected number of bytes
// was received.
static bool readSensorWord(int &word, int &temp)
{
  Wire.requestFrom(data.TWIadd, kSensorBytes);
  delay(10);

  word = 0;
  temp = 0;
  int i = 0;
  while(Wire.available() > 0)
  {
    uint8_t c = Wire.read();
    if(i == 1) word |= c << 8;
    if(i == 2) word |= c;
    if(i == 4) temp |= c << 8;
    if(i == 5) temp |= c;
    i++;
  }
  return i == kSensorBytes;
}

bool readPressure(void *)
{
  // Read the factory-calibrated pressure (default register, no command byte).
  Wire.beginTransmission(data.TWIadd);
  Wire.endTransmission(true);
  int press = 0, scratchTemp = 0;
  readSensorWord(press, scratchTemp);

  // Read the raw pressure/temperature counts (register 0xD0).
  Wire.beginTransmission(data.TWIadd);
  Wire.write(0xD0);
  Wire.endTransmission(true);
  readSensorWord(data.rawData, data.rawTemp);

  // Convert sensor counts to Torr, using either the optional PWL calibration
  // table or the factory formula.
  float fval;
  if(data.useCalTable)
  {
    // Optional calibration: interpolate pressure from the raw count.
    fval = pwlPressure(data.rawData);
  }
  else
  {
    // Factory conversion:
    //   0..50000      : counts are microns directly.
    //   50000..65535  : trend-only region, ~45.7 microns per count above the knee.
    fval = press;
    if(fval > kTrendKnee)
      fval = kTrendKnee + (fval - kTrendKnee) * kMicronsPerCount;
    fval = fval / 1000.0f;   // microns -> Torr (1 Torr = 1000 microns)
  }

  // Apply the appropriate user zero offset.
  if(fval > kTorrThreshold) fval += data.offsetTorr;
  else                      fval += (float)data.offsetmTorr / 1000.0f;

  // Absolute pressure can never be negative. A large negative zero-offset (or
  // sensor noise near full vacuum) can drive the computed value below zero, so
  // clamp to the physical floor before reporting/displaying it.
  if(fval < kMinPressure) fval = kMinPressure;

  data.calPress = (double)fval;
  displayPressure(fval);
  return true;   // keep the timer scheduled
}

// ---------------------------------------------------------------------------
// Persist-on-change (periodic task)
// ---------------------------------------------------------------------------

bool saveChanges(void *)
{
  if(settingsChanged())
  {
    lastSaved = data;
    saveData();
  }
  return true;   // keep the timer scheduled
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------

void setup()
{
  delay(100);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("GAACE", 0, 0, 1);
  tft.setTextFont(4);

  data.offsetTorr  = 0;
  data.offsetmTorr = 0;

  // 115200 is conventional; on the ESP32-S3 USB-CDC port the baud rate is
  // ignored, but this keeps the firmware correct if Serial maps to a UART.
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  cp.registerStream(&Serial);
  cp.registerCommands(&cmdList);
  cp.registerCommands(dbg.debugCommands());
  dbg.registerDebugFunction(Debug);

  Wire.begin(kI2C_SDA, kI2C_SCL, kI2C_Hz);
  buttonA.begin();
  buttonB.begin();

  // Mount the filesystem once, then load (or create) the settings file.
  spiffsReady = SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED);
  if(!loadData()) saveData();
  lastSaved = data;

  timer.every(kReadIntervalMs, readPressure);
  timer.every(kSaveIntervalMs, saveChanges);
}

void loop()
{
  timer.tick();
  cp.processStreams();
  cp.processCommands();

  // Sample both buttons once per loop. The Button library's pressed() only
  // inspects the snapshot taken by read(); without these calls the debounced
  // state never updates and pressed() can never return true.
  buttonA.read();
  buttonB.read();

  // Button A nudges the zero offset up; Button B nudges it down. Which trim is
  // adjusted (coarse Torr vs fine mTorr) follows the current reading range.
  if(buttonA.pressed())
  {
    if(data.calPress > kTorrThreshold) data.offsetTorr++;
    else                               data.offsetmTorr += 10;
  }
  if(buttonB.pressed())
  {
    if(data.calPress > kTorrThreshold) data.offsetTorr--;
    else                               data.offsetmTorr -= 10;
  }

  // Keep both trims within range no matter how they were changed (button press
  // or ?TOFFSET / ?MTOFFSET serial command).
  data.offsetTorr  = constrain(data.offsetTorr,  -kMaxOffsetTorr,  kMaxOffsetTorr);
  data.offsetmTorr = constrain(data.offsetmTorr, -kMaxOffsetmTorr, kMaxOffsetmTorr);
}
