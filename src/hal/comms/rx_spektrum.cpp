#include "hal/comms/rx_spektrum.h"
#include "config.h"
#include "math/utils.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

RCData rc_data; ////global variable to hold the latest RC data read from the receiver

enum rx_input_t {
#ifdef USE_ESC
    RX_ESC_INP = 0,
#endif
#ifdef USE_AILERON
    RX_AILERON_INP,
#endif
#ifdef USE_ELEVATOR
    RX_ELEVATOR_INP,
#endif
#ifdef USE_RUDDER
    RX_RUDDER_INP,
#endif
    RX_MODE_INP,
    NUM_RX_CHANNELS
};

const int INPUT_GPIOS[NUM_RX_CHANNELS] = {
#ifdef USE_ESC
    rx_esc_pin,
#endif
#ifdef USE_AILERON
    rx_aileron_pin,
#endif
#ifdef USE_ELEVATOR
    rx_elevator_pin,
#endif
#ifdef USE_RUDDER
    rx_rudder_pin,
#endif
    rx_mode_pin
};

volatile unsigned int pulse_widths[NUM_RX_CHANNELS] = {0};
uint32_t rise_time[NUM_RX_CHANNELS] = {0};

static bool is_healthy_pwm() {
    int i = 0;
    for(i=0; i<NUM_RX_CHANNELS; i++){
        if (pulse_widths[i] < SERVO_MIN || pulse_widths[i] > SERVO_MAX) {
            return false;
        }
    }
    return true;
}

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
            if (i < 3) {
                MCPWM0.int_clr.val = (1 << (MCPWM_CAP0_INT_CLR_S + sig));
            } else {
                MCPWM1.int_clr.val = (1 << (MCPWM_CAP0_INT_CLR_S + sig));
            }
        }
    }
    rc_data.healthy = is_healthy_pwm();
#ifdef USE_AILERON
    rc_data.aileron_pwm = pulse_widths[RX_AILERON_INP];
#endif
#ifdef USE_ELEVATOR
    rc_data.elevator_pwm = pulse_widths[RX_ELEVATOR_INP];
#endif
#ifdef USE_RUDDER
    rc_data.rudder_pwm = pulse_widths[RX_RUDDER_INP];
#endif
#ifdef USE_ESC
    rc_data.throttle_pwm = pulse_widths[RX_ESC_INP];
#endif
    rc_data.flightmode_pwm = pulse_widths[RX_MODE_INP];
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

float rx_to_angle(float raw_pwm, float max_angle){
    
    raw_pwm = math::clamp_value(raw_pwm, 1000.0f, 2000.0f);
    return ((raw_pwm-1500.0f)/500.0f) * max_angle;
    
}

float rx_to_throttle(float raw_pwm){
    if (raw_pwm < 1050) {
        raw_pwm = 1000;
    } else if (raw_pwm > 2000) {
        raw_pwm = 2000;
    }
    return (raw_pwm - 1000.0f) / 10.0f;   // 0-100 percent of stick
}

float get_des_roll() {
    float des_roll = rx_to_angle(rc_data.aileron_pwm, max_roll_angle); //////////////////outputs angles
    return des_roll;
}

float get_des_pitch() {
    float des_pitch = rx_to_angle(rc_data.elevator_pwm, max_pitch_angle);
    return des_pitch;
}
float get_des_yaw() {
    float des_yaw = rx_to_angle(rc_data.rudder_pwm, max_yaw_angle);
    return des_yaw;
}

float get_des_throttle(){
    float des_throttle = rx_to_throttle(rc_data.throttle_pwm);
    return des_throttle;
}

float get_flight_mode_pwm() {
    return rc_data.flightmode_pwm;
}

