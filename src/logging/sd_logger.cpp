#include "logging/sd_logger.h"
#include "config.h"

#include <SPI.h>
#include <SD.h>

// Create a dedicated SPI object for the SD card
SPIClass featherSPI(HSPI);

File logFile; // Global file object for the current log

bool SD_Logger_Init() {
    Serial.println("\n--- Adafruit Feather V2 SD Init ---");

    // Force the SPI bus to use the physical pins printed on the Feather
    featherSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

    // Initialize the SD card using the custom SPI bus.
    // Drop the frequency to 4MHz (4000000) to maximize stability.
    if (!SD.begin(SD_CS, featherSPI, 4000000)) {
        Serial.println("Card Mount Failed.");
        Serial.println("Troubleshooting: Check wiring, SD format (FAT32), or power supply.");
        return false;
    }
    
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        Serial.println("No SD card attached (Hardware detected, but no card found).");
        return false;
    }

    Serial.println("SD Card Successfully Mounted!");
    
    Serial.print("Card Type: ");
    if (cardType == CARD_MMC) Serial.println("MMC");
    else if (cardType == CARD_SD) Serial.println("SDSC");
    else if (cardType == CARD_SDHC) Serial.println("SDHC");
    else Serial.println("UNKNOWN");

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("Card Size: %lluMB\n", cardSize);

    return true;
}

bool SD_Logger_CreateNewLog() {
    // Find an unused log file name
    char filename[] = "/log_000.csv";
    for (int i = 0; i < 1000; i++) {
        filename[5] = i / 100 + '0';
        filename[6] = (i / 10) % 10 + '0';
        filename[7] = i % 10 + '0';
        if (!SD.exists(filename)) {
            logFile = SD.open(filename, FILE_WRITE);
            if (logFile) {
                Serial.printf("Created new log file: %s\n", filename);
                return true;
            } else {
                Serial.printf("Failed to create log file: %s\n", filename);
                return false;
            }
        }
    }
    Serial.println("Could not create a new log file (all 000-999 exist).");
    return false;
}

void SD_Logger_WriteHeader() {
    if (!logFile) {
        return;
    }
    logFile.println("roll,pitch,yaw,altitude,des_altitude,gps_lat,gps_long,gps_alt,gps_speed,gps_heading,gps_sats,gps_fix_quality,gps_lock_acquired,baro_altitude,flightmode,waypoint_distance,waypoint_heading,waypoint_target_alt,waypoint_leg_progress,waypoint_mission_progress,waypoint_index,waypoint_total,waypoint_mission_complete,waypoint_target_lat,waypoint_target_lon");
    logFile.flush(); // Ensure header is written immediately
}

void SD_Logger_LogData(const telemetrydata& data) {
    if (!logFile) {
        return;
    }

    logFile.print(data.roll, 2); logFile.print(",");
    logFile.print(data.pitch, 2); logFile.print(",");
    logFile.print(data.yaw, 2); logFile.print(",");
    logFile.print(data.altitude, 2); logFile.print(",");
    logFile.print(data.des_altitude, 2); logFile.print(",");
    logFile.print(data.gps_lat, 6); logFile.print(",");
    logFile.print(data.gps_long, 6); logFile.print(",");
    logFile.print(data.gps_alt, 2); logFile.print(",");
    logFile.print(data.gps_speed, 2); logFile.print(",");
    logFile.print(data.gps_heading, 2); logFile.print(",");
    logFile.print(data.gps_sats); logFile.print(",");
    logFile.print(data.gps_fix_quality); logFile.print(",");
    logFile.print(data.gps_lock_acquired); logFile.print(",");
    logFile.print(data.baro_altitude, 2); logFile.print(",");
    logFile.print(data.flightmode); logFile.print(",");
    logFile.print(data.waypoint_distance, 2); logFile.print(",");
    logFile.print(data.waypoint_heading, 2); logFile.print(",");
    logFile.print(data.waypoint_target_alt, 2); logFile.print(",");
    logFile.print(data.waypoint_leg_progress, 2); logFile.print(",");
    logFile.print(data.waypoint_mission_progress, 2); logFile.print(",");
    logFile.print(data.waypoint_index); logFile.print(",");
    logFile.print(data.waypoint_total); logFile.print(",");
    logFile.print(data.waypoint_mission_complete); logFile.print(",");
    logFile.print(data.waypoint_target_lat, 6); logFile.print(",");
    logFile.print(data.waypoint_target_lon, 6);
    logFile.println();
}

void SD_Logger_Flush() {
    if (logFile) {
        logFile.flush();
    }
}

void SD_Logger_CloseLog() {
    if (logFile) {
        logFile.close();
        Serial.println("Log file closed.");
    }
}