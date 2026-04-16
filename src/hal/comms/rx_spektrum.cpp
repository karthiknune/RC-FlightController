#include <limits>
#include "hal/comms/rx_spektrum.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "config.h"

/// TODO: delete after writing rx_read()
float raw_aileron_pwm = 1500;
float raw_elevator_pwm = 1500;
float raw_throttle_pwm = 1000;
float raw_flightmode_pwm = 1900;

enum rx_input_t {
    RX_ESC_INP = 0,
    RX_ELEVATOR_INP,
    RX_RUDDER_INP,
    RX_MODE_INP
};

const int INPUT_GPIOS[NUM_RX_CHANNELS] = {
    rx_esc_pin,
    rx_elevator_pin,
    rx_rudder_pin,
    rx_mode_pin
};

volatile uint32_t pulse_widths[NUM_RX_CHANNELS] = {0};
uint32_t rise_time[NUM_RX_CHANNELS] = {0};

static void IRAM_ATTR isr_handler(void *arg) {
    uint32_t status0 = MCPWM0.int_st.val;   // ESC, Elevator, Rudder
    uint32_t status1 = MCPWM1.int_st.val;   // Flight Mode

    for (int i = 0; i < NUM_RX_CHANNELS; i++) {
        mcpwm_unit_t unit = (i < 3) ? MCPWM_UNIT_0 : MCPWM_UNIT_1;
        mcpwm_capture_signal_t sig = (mcpwm_capture_signal_t)(i % 3);
        uint32_t active_status = (i < 3) ? status0 : status1;

        // Check if the specific capture interrupt bit is set
        if (active_status & (1 << (MCPWM_CAP0_INT_ST_S + sig))) {
            uint32_t edge_type = mcpwm_capture_signal_get_edge(unit, sig);
            uint32_t val = mcpwm_capture_signal_get_value(unit, sig);

            if (edge_type == MCPWM_NEG_EDGE) {
                rise_time[i] = val;
            } else {
                // Microseconds = Ticks / (Clock Frequency in MHz)
                pulse_widths[i] = (val - rise_time[i]) / 80; 
            }
            
            // Clear the interrupt manually
            if (i < 3) MCPWM0.int_clr.val = (1 << (MCPWM_CAP0_INT_CLR_S + sig));
            else MCPWM1.int_clr.val = (1 << (MCPWM_CAP0_INT_CLR_S + sig));
        }
    }
}

void rx_init() {
    // Init GPIOs
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, INPUT_GPIOS[0]);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_1, INPUT_GPIOS[1]);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_2, INPUT_GPIOS[2]);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM_CAP_0, INPUT_GPIOS[3]);

    // Enable Capture on both edges
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_BOTH_EDGE, 0);
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP1, MCPWM_BOTH_EDGE, 0);
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP2, MCPWM_BOTH_EDGE, 0);
    mcpwm_capture_enable(MCPWM_UNIT_1, MCPWM_SELECT_CAP0, MCPWM_BOTH_EDGE, 0);

    // Register ISR
    mcpwm_isr_register(MCPWM_UNIT_0, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);
    mcpwm_isr_register(MCPWM_UNIT_1, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);

    // Enable the interrupt bit in the hardware
    MCPWM0.int_ena.val |= (MCPWM_CAP0_INT_ENA | MCPWM_CAP1_INT_ENA | MCPWM_CAP2_INT_ENA);
    MCPWM1.int_ena.val |= (MCPWM_CAP0_INT_ENA);
}

// Gives Receiver Throttole as percentage (0-100%)
float get_des_throttle() {
  if(pulse_widths[RX_ESC_INP] < SERVO_MIN) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  return (((long)pulse_widths[RX_ESC_INP] - THROTTLE_INT) / THROTTLE_SLOPE);
}

// Gives Receiever Aileron Deflection Angle in Degrees
float get_des_pitch() {
  if(pulse_widths[RX_ELEVATOR_INP] < SERVO_MIN) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  return (((long)pulse_widths[RX_ELEVATOR_INP] - ELEVATOR_INT) / ELEVATOR_SLOPE);
}

float get_des_roll() {
    return std::numeric_limits<float>::quiet_NaN();
}

// Gives Receiver Rudder Deflection Angle in Degrees
float get_des_yaw() {
  if(pulse_widths[RX_RUDDER_INP] < SERVO_MIN) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  return (((long)pulse_widths[RX_RUDDER_INP] - RUDDER_INT) / RUDDER_SLOPE);
}

// Gives Receiver Requested Flight Mode
bool get_auto_mode() {
    return (pulse_widths[RX_MODE_INP] >= AUTO_MODE_THRESHOLD);
}