#include "hal/sensors/gps.h"

#include <HardwareSerial.h>
#include <Wire.h>

#include <cmath>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "config.h"
#include "hal/sensors/sensor_bus.h"

namespace {

HardwareSerial gps_serial(GPS_UART_NUM);
char gps_sentence_buffer[GPS_SENTENCE_BUFFER_SIZE] = {};
size_t gps_sentence_length = 0;
constexpr float kKnotsToMetersPerSecond = 0.514444f;
constexpr float kRadiansToDegrees = 57.2957795f;

// QMC5883L register map.
constexpr uint8_t kQmcRegData = 0x00;
constexpr uint8_t kQmcRegControl1 = 0x09;
constexpr uint8_t kQmcRegControl2 = 0x0A;
constexpr uint8_t kQmcRegSetReset = 0x0B;
// CTRL1 = MODE(continuous=0x01) | ODR(200Hz=0x0C) | RNG(8G=0x10) | OSR(512=0x00).
constexpr uint8_t kQmcCtrl1Continuous = 0x1D;
constexpr uint8_t kQmcCtrl2SoftReset = 0x80;
constexpr uint8_t kQmcSetResetPeriodValue = 0x01;

bool g_compass_ready = false;
bool g_compass_missing_logged = false;
uint32_t g_compass_last_init_attempt_ms = 0;

void clear_fix_data(GPSData &data) {
    data.latitude = 0.0;
    data.longitude = 0.0;
    data.altitude = 0.0f;
    data.raw_coordinates.latitude[0] = '\0';
    data.raw_coordinates.latitude_dir = '\0';
    data.raw_coordinates.longitude[0] = '\0';
    data.raw_coordinates.longitude_dir = '\0';
    data.lock_acquired = false;
}

void clear_compass_data(GPSData &data) {
    data.mag_x = 0.0f;
    data.mag_y = 0.0f;
    data.mag_z = 0.0f;
    data.compass_heading = 0.0f;
    data.compass_healthy = false;
}

float normalize_heading_degrees(float heading_degrees) {
    while (heading_degrees < 0.0f) {
        heading_degrees += 360.0f;
    }
    while (heading_degrees >= 360.0f) {
        heading_degrees -= 360.0f;
    }
    return heading_degrees;
}

void update_compass_availability(bool available) {
    if (available) {
        if (!g_compass_ready && SENSOR_STATUS_LOGGING_ENABLED) {
            Serial.println("QMC5883L (GPS compass) available.");
        }
        g_compass_ready = true;
        g_compass_missing_logged = false;
        return;
    }

    if ((g_compass_ready || !g_compass_missing_logged) && SENSOR_STATUS_LOGGING_ENABLED) {
        Serial.println("QMC5883L unavailable. Continuing without GPS compass.");
    }
    g_compass_ready = false;
    g_compass_missing_logged = true;
}

uint8_t write_compass_register_locked(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(GPS_COMPASS_I2C_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission(true);
}

bool send_compass_soft_reset_locked() {
    const uint8_t err = write_compass_register_locked(kQmcRegControl2, kQmcCtrl2SoftReset);
    if (err != 0) {
        Serial.printf("[QMC5883L] soft-reset write failed, endTransmission=%u\n", err);
        return false;
    }
    return true;
}

bool finalize_compass_config_locked() {
    uint8_t err = write_compass_register_locked(kQmcRegSetReset, kQmcSetResetPeriodValue);
    if (err != 0) {
        Serial.printf("[QMC5883L] SET/RESET period write failed, endTransmission=%u\n", err);
        return false;
    }
    err = write_compass_register_locked(kQmcRegControl1, kQmcCtrl1Continuous);
    if (err != 0) {
        Serial.printf("[QMC5883L] CTRL1 write failed, endTransmission=%u\n", err);
        return false;
    }
    Serial.println("[QMC5883L] config writes OK.");
    return true;
}

bool try_initialize_compass() {
    if (!GPS_COMPASS_ENABLED) {
        return false;
    }

    const uint32_t now_ms = millis();
    if (g_compass_last_init_attempt_ms != 0 &&
        (now_ms - g_compass_last_init_attempt_ms) < SENSOR_RECONNECT_INTERVAL_MS) {
        return g_compass_ready;
    }
    g_compass_last_init_attempt_ms = now_ms;

    if (!SensorBus_Init()) {
        update_compass_availability(false);
        return false;
    }

    if (!SensorBus_Lock(pdMS_TO_TICKS(SENSOR_I2C_LOCK_TIMEOUT_MS))) {
        return false;
    }
    Wire.setClock(I2C_BUS_FREQUENCY_HZ);
    const bool reset_ok = send_compass_soft_reset_locked();

    if (reset_ok) {
        // IMPORTANT: Do NOT unlock the bus during this delay!
        // If the IMU task grabs the bus and sends 400kHz traffic while
        // the compass is rebooting, the compass will permanently lock up.
        // Keep this under 20ms to avoid starving the IMU/Baro of the mutex lock.
        delay(10);

        Wire.setClock(I2C_BUS_FREQUENCY_HZ);
        const bool ok = finalize_compass_config_locked();
        SensorBus_Unlock();
        update_compass_availability(ok);
        return ok;
    }

    SensorBus_Unlock();
    update_compass_availability(false);
    return false;
}

bool read_compass_raw_locked(int16_t &raw_x, int16_t &raw_y, int16_t &raw_z, bool &data_ready) {
    data_ready = false;

    // 1. Check Status Register (0x06) for Data Ready (Bit 0)
    Wire.beginTransmission(GPS_COMPASS_I2C_ADDRESS);
    Wire.write(0x06);
    // Force a STOP bit instead of a Repeated Start to clear ESP32 bus capacitance
    if (Wire.endTransmission(true) != 0) {
        return false;
    }
    if (Wire.requestFrom(static_cast<uint8_t>(GPS_COMPASS_I2C_ADDRESS), static_cast<uint8_t>(1)) != 1) {
        return false;
    }
    if ((Wire.read() & 0x01) == 0) {
        // I2C is healthy, but no new measurement is ready yet.
        return true;
    }

    // 2. Read 6 data registers
    Wire.beginTransmission(GPS_COMPASS_I2C_ADDRESS);
    Wire.write(kQmcRegData);
    // Force a STOP bit instead of a Repeated Start
    if (Wire.endTransmission(true) != 0) {
        return false;
    }
    if (Wire.requestFrom(static_cast<uint8_t>(GPS_COMPASS_I2C_ADDRESS),
                         static_cast<uint8_t>(6)) != 6) {
        return false;
    }
    const uint8_t x_lsb = Wire.read();
    const uint8_t x_msb = Wire.read();
    const uint8_t y_lsb = Wire.read();
    const uint8_t y_msb = Wire.read();
    const uint8_t z_lsb = Wire.read();
    const uint8_t z_msb = Wire.read();
    raw_x = static_cast<int16_t>((x_msb << 8) | x_lsb);
    raw_y = static_cast<int16_t>((y_msb << 8) | y_lsb);
    raw_z = static_cast<int16_t>((z_msb << 8) | z_lsb);
    data_ready = true;
    return true;
}

void read_compass(GPSData &data) {
    if (!g_compass_ready && !try_initialize_compass()) {
        data.compass_healthy = false;
        return;
    }

    if (!SensorBus_Lock(pdMS_TO_TICKS(SENSOR_I2C_LOCK_TIMEOUT_MS))) {
        data.compass_healthy = false;
        return;
    }
    Wire.setClock(I2C_BUS_FREQUENCY_HZ);

    int16_t raw_x = 0;
    int16_t raw_y = 0;
    int16_t raw_z = 0;
    bool data_ready = false;
    const bool ok = read_compass_raw_locked(raw_x, raw_y, raw_z, data_ready);
    SensorBus_Unlock();

    if (!ok) {
        update_compass_availability(false);
        data.compass_healthy = false;
        return;
    }

    if (!data_ready) {
        // Compass is healthy, just use the previously recorded values until a new reading is available
        data.compass_healthy = true;
        return;
    }

    const float calibrated_x =
        (static_cast<float>(raw_x) - GPS_COMPASS_OFFSET_X) * GPS_COMPASS_SCALE_X;
    const float calibrated_y =
        (static_cast<float>(raw_y) - GPS_COMPASS_OFFSET_Y) * GPS_COMPASS_SCALE_Y;
    const float calibrated_z =
        (static_cast<float>(raw_z) - GPS_COMPASS_OFFSET_Z) * GPS_COMPASS_SCALE_Z;

    data.mag_x = calibrated_x;
    data.mag_y = calibrated_y;
    data.mag_z = calibrated_z;

    const float heading_rad = std::atan2(calibrated_y, calibrated_x);
    const float heading_deg = (heading_rad * kRadiansToDegrees) + GPS_COMPASS_DECLINATION_DEG;
    data.compass_heading = normalize_heading_degrees(heading_deg);
    data.compass_healthy = true;
}

size_t split_nmea_fields(char *sentence, char *fields[], size_t max_fields) {
    if (sentence == nullptr || fields == nullptr || max_fields == 0) {
        return 0;
    }

    size_t field_count = 0;
    fields[field_count++] = sentence;

    for (char *cursor = sentence; *cursor != '\0' && field_count < max_fields; ++cursor) {
        if (*cursor == '*') {
            *cursor = '\0';
            break;
        }

        if (*cursor == ',') {
            *cursor = '\0';
            fields[field_count++] = cursor + 1;
        }
    }

    return field_count;
}

bool is_gga_sentence(const char *sentence_type) {
    if (sentence_type == nullptr) {
        return false;
    }

    return std::strcmp(sentence_type, "$GPGGA") == 0 ||
           std::strcmp(sentence_type, "$GNGGA") == 0;
}

bool is_rmc_sentence(const char *sentence_type) {
    if (sentence_type == nullptr) {
        return false;
    }

    return std::strcmp(sentence_type, "$GPRMC") == 0 ||
           std::strcmp(sentence_type, "$GNRMC") == 0;
}

bool parse_int_field(const char *field, int &value) {
    if (field == nullptr || *field == '\0') {
        return false;
    }

    char *end = nullptr;
    const long parsed = std::strtol(field, &end, 10);
    if (end == field || *end != '\0') {
        return false;
    }

    value = static_cast<int>(parsed);
    return true;
}

bool parse_float_field(const char *field, float &value) {
    if (field == nullptr || *field == '\0') {
        return false;
    }

    char *end = nullptr;
    const float parsed = std::strtof(field, &end);
    if (end == field || *end != '\0') {
        return false;
    }

    value = parsed;
    return true;
}

bool parse_local_time(const char *field, GPSLocalTime &local_time) {
    local_time.hour = 0;
    local_time.minute = 0;
    local_time.second = 0;
    local_time.valid = false;

    if (field == nullptr || std::strlen(field) < 6) {
        return false;
    }

    for (int index = 0; index < 6; ++index) {
        if (!std::isdigit(static_cast<unsigned char>(field[index]))) {
            return false;
        }
    }

    const int utc_hour = ((field[0] - '0') * 10) + (field[1] - '0');
    const int minutes = ((field[2] - '0') * 10) + (field[3] - '0');
    const int seconds = ((field[4] - '0') * 10) + (field[5] - '0');

    int local_hour = utc_hour + GPS_TIME_ZONE_OFFSET;
    while (local_hour < 0) {
        local_hour += 24;
    }
    while (local_hour >= 24) {
        local_hour -= 24;
    }

    local_time.hour = local_hour;
    local_time.minute = minutes;
    local_time.second = seconds;
    local_time.valid = true;
    return true;
}

bool parse_coordinate(const char *field, char direction, double &coordinate) {
    if (field == nullptr || *field == '\0' || direction == '\0') {
        return false;
    }

    char *end = nullptr;
    const double nmea_coordinate = std::strtod(field, &end);
    if (end == field || *end != '\0') {
        return false;
    }

    const double degrees = std::floor(nmea_coordinate / 100.0);
    const double minutes = nmea_coordinate - (degrees * 100.0);
    coordinate = degrees + (minutes / 60.0);

    const char normalized_direction =
        static_cast<char>(std::toupper(static_cast<unsigned char>(direction)));
    if (normalized_direction == 'S' || normalized_direction == 'W') {
        coordinate = -coordinate;
    }

    return normalized_direction == 'N' || normalized_direction == 'S' ||
           normalized_direction == 'E' || normalized_direction == 'W';
}

void copy_coordinate_field(char *destination, size_t destination_size, const char *source) {
    if (destination == nullptr || destination_size == 0) {
        return;
    }

    std::snprintf(destination, destination_size, "%s", source == nullptr ? "" : source);
}

bool parse_gga_sentence(char *sentence, GPSData &data) {
    char *fields[GPS_MAX_FIELDS] = {};
    const size_t field_count = split_nmea_fields(sentence, fields, GPS_MAX_FIELDS);

    if (field_count <= 14 || !is_gga_sentence(fields[0])) {
        return false;
    }

    data.healthy = true;
    parse_local_time(fields[1], data.local_time);

    int fix_quality = 0;
    parse_int_field(fields[6], fix_quality);
    data.fix_quality = fix_quality;
    // 0=no fix, 1=GPS SPS, 2=DGPS, 3=PPS, 4=RTK fixed, 5=Float RTK, 6=dead reckoning
    data.lock_acquired = (fix_quality >= 1 && fix_quality <= 3);

    int satellites = 0;
    parse_int_field(fields[7], satellites);
    data.satellites = satellites;

    if (!data.lock_acquired) {
        clear_fix_data(data);
        data.fix_quality = fix_quality;
        data.satellites = satellites;
        return true;
    }

    const char latitude_direction = (fields[3] != nullptr && *fields[3] != '\0') ? fields[3][0] : '\0';
    const char longitude_direction = (fields[5] != nullptr && *fields[5] != '\0') ? fields[5][0] : '\0';

    double latitude = 0.0;
    double longitude = 0.0;
    if (!parse_coordinate(fields[2], latitude_direction, latitude) ||
        !parse_coordinate(fields[4], longitude_direction, longitude)) {
        clear_fix_data(data);
        data.fix_quality = fix_quality;
        data.satellites = satellites;
        return true;
    }

    data.latitude = latitude;
    data.longitude = longitude;
    parse_float_field(fields[9], data.altitude);
    copy_coordinate_field(data.raw_coordinates.latitude, sizeof(data.raw_coordinates.latitude), fields[2]);
    copy_coordinate_field(data.raw_coordinates.longitude, sizeof(data.raw_coordinates.longitude), fields[4]);
    data.raw_coordinates.latitude_dir = latitude_direction;
    data.raw_coordinates.longitude_dir = longitude_direction;
    data.lock_acquired = true;

    return true;
}

bool parse_rmc_sentence(char *sentence, GPSData &data) {
    char *fields[GPS_MAX_FIELDS] = {};
    const size_t field_count = split_nmea_fields(sentence, fields, GPS_MAX_FIELDS);

    if (field_count <= 8 || !is_rmc_sentence(fields[0])) {
        return false;
    }

    data.healthy = true;
    parse_local_time(fields[1], data.local_time);

    const bool valid_fix = fields[2] != nullptr &&
                           (fields[2][0] == 'A' || fields[2][0] == 'a');
    if (!valid_fix) {
        data.speed = 0.0f;
        data.heading = 0.0f;
        return true;
    }

    float speed_knots = 0.0f;
    float track_heading = 0.0f;
    parse_float_field(fields[7], speed_knots);
    parse_float_field(fields[8], track_heading);

    data.speed = speed_knots * kKnotsToMetersPerSecond;
    data.heading = track_heading;
    return true;
}

bool process_incoming_byte(char incoming_byte, GPSData &data) {
    if (incoming_byte == '\r') {
        return false;
    }

    if (incoming_byte == '\n') {
        if (gps_sentence_length == 0) {
            return false;
        }

        gps_sentence_buffer[gps_sentence_length] = '\0';
        bool parsed_sentence = false;
        if (parse_gga_sentence(gps_sentence_buffer, data)) {
            parsed_sentence = true;
            if (GPS_DEBUG_OUTPUT_ENABLED) {
                GPS_PrintStatus(Serial, data);
            }
        } else if (parse_rmc_sentence(gps_sentence_buffer, data)) {
            parsed_sentence = true;
        }

        gps_sentence_length = 0;
        return parsed_sentence;
    }

    if (gps_sentence_length >= (GPS_SENTENCE_BUFFER_SIZE - 1)) {
        gps_sentence_length = 0;
        return false;
    }

    gps_sentence_buffer[gps_sentence_length++] = incoming_byte;
    return false;
}

} // namespace

void GPS_Init() {
    gps_serial.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    gps_sentence_length = 0;
    gps_sentence_buffer[0] = '\0';

    clear_compass_data(gps_data);

    if (GPS_DEBUG_OUTPUT_ENABLED) {
        Serial.printf("Initializing GPS on UART%d at %lu baud...\n", GPS_UART_NUM, GPS_BAUD_RATE);
        Serial.println("Waiting for data stream...\n");
    }

    if (GPS_COMPASS_ENABLED) {
        (void)try_initialize_compass();
    }
}

bool GPS_Read(GPSData &data) {
    bool parsed_gga = false;

    while (gps_serial.available() > 0) {
        if (process_incoming_byte(static_cast<char>(gps_serial.read()), data)) {
            parsed_gga = true;
        }
    }

    if (GPS_COMPASS_ENABLED) {
        read_compass(data);
    }

    return parsed_gga;
}

void GPS_PrintStatus(Stream &stream, const GPSData &data) {
    stream.println("========================================");

    if (data.local_time.valid) {
        stream.printf(
            "Local Time  : %02d:%02d:%02d (UTC %+d)\n",
            data.local_time.hour,
            data.local_time.minute,
            data.local_time.second,
            GPS_TIME_ZONE_OFFSET
        );
    } else if (data.healthy) {
        stream.println("Local Time  : Syncing...");
    } else {
        stream.println("Local Time  : Waiting for sync...");
    }

    if (data.lock_acquired) {
        stream.printf("Status      : LOCKED (%02d satellites)\n", data.satellites);
        stream.printf(
            "Coordinates : %.6f, %.6f\n",
            data.latitude,
            data.longitude
        );
        stream.printf(
            "Raw NMEA    : %s %c, %s %c\n",
            data.raw_coordinates.latitude,
            data.raw_coordinates.latitude_dir,
            data.raw_coordinates.longitude,
            data.raw_coordinates.longitude_dir
        );
    } else {
        stream.println("Status      : Acquiring satellites...");
        stream.println("Coordinates : Waiting for lock...");
    }

    if (data.compass_healthy) {
        stream.printf(
            "Compass     : %.1f deg  (mag x=%.0f y=%.0f z=%.0f)\n",
            data.compass_heading,
            data.mag_x,
            data.mag_y,
            data.mag_z
        );
    } else if (GPS_COMPASS_ENABLED) {
        stream.println("Compass     : Unavailable");
    }

    stream.println("========================================");
    stream.println();
}
