#pragma once
#include "datatypes.h"

class AHRS
{
private:
    unsigned long last_time;
    bool is_initialized;

public:
    AHRS();
    void init();

    // Takes the raw hardware data, performs the filter, and updates the filtered struct
    void update(const IMUData_raw &raw_data, IMUData_filtered &filtered_data);
};