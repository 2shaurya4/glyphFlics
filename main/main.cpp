/*
 * Drink Dispenser: Fluid Sim + VL53L0X + Multi-Pump Control
 *
 * - LED matrix shows fluid level in a tank with sloshing physics
 * - When a cup is held within 100mm of the ToF sensor for 3 seconds,
 *   the ESP reads GPIO 1 & 2 to select which pump to activate:
 *     GPIO2=0, GPIO1=1 → Pump A (GPIO 18)
 *     GPIO2=1, GPIO1=0 → Pump B (GPIO 20)
 *     GPIO2=1, GPIO1=1 → Pump C (GPIO 19)
 *     GPIO2=0, GPIO1=0 → No selection, ignored
 * - Fluid drains 10% per dispense, then pump turns off
 * - BNO055 IMU drives sloshing/splash physics
 *
 * I2C addresses:
 *   BNO055  = 0x28  (ADR pin → GND)
 *   VL53L0X = 0x29  (default)
 *
 * Build:
 *   idf.py set-target esp32c6
 *   idf.py build
 *   idf.py -p COMx flash monitor
 */

#include <cstdio>
#include <cstring>
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "led_strip.h"
#include "bno055.h"
#include "VL53L0X.h"
#include "i2c.h"

static const char *TAG = "dispenser";

// =============================================================================
// Hardware config
// =============================================================================

#define LED_GPIO        21
#define I2C_SDA_GPIO    22
#define I2C_SCL_GPIO    23

// Pump outputs
#define PUMP_A_GPIO     18      // GPIO2=0, GPIO1=1
#define PUMP_B_GPIO     20      // GPIO2=1, GPIO1=0
#define PUMP_C_GPIO     19      // GPIO2=1, GPIO1=1

// Drink selection inputs
#define SELECT_GPIO_1    1
#define SELECT_GPIO_2    2

#define BNO055_ADDR     0x28
#define VL53L0X_ADDR    0x29

// =============================================================================
// LED matrix
// =============================================================================

#define MATRIX_COLS     10
#define MATRIX_ROWS     8
#define NUM_LEDS        (MATRIX_COLS * MATRIX_ROWS)
#define SERPENTINE      0
#define BRIGHTNESS      20

// =============================================================================
// BNO055 startup
// =============================================================================

#define BNO055_INIT_RETRIES     10
#define BNO055_RETRY_DELAY_MS   500

// =============================================================================
// Simulation tuning
// =============================================================================

#define SIM_FPS         60

#define WAVE_SPEED      0.15f
#define ACCEL_SCALE     0.006f
#define DAMPING         0.995f

// Splash particles
#define MAX_PARTICLES       30
#define SPLASH_THRESHOLD    12.0f
#define SPLASH_HEAVY        20.0f
#define SPAWN_RATE          3
#define PARTICLE_GRAVITY    0.08f
#define PARTICLE_DAMPING    0.98f
#define SPLASH_SPEED_MIN    0.3f
#define SPLASH_SPEED_MAX    1.2f
#define SPLASH_HSPEED       0.15f
#define DROPLET_TRAIL       0.4f
#define DROPLET_WATER       0.05f
#define SURFACE_GLOW        0

// =============================================================================
// Dispense config
// =============================================================================

#define CUP_DETECT_DIST_MM      100
#define CUP_HOLD_TIME_MS        3000
#define DISPENSE_PERCENT         10
#define PUMP_RUN_TIME_MS        2000
#define COOLDOWN_TIME_MS        2000

// =============================================================================
// Shared state
// =============================================================================

static volatile uint16_t g_tof_distance_mm = 0;
static volatile bool     g_tof_valid = false;

typedef enum {
    DISPENSE_IDLE,
    DISPENSE_CUP_DETECTED,
    DISPENSE_PUMPING,
    DISPENSE_COOLDOWN
} dispense_state_t;

static volatile dispense_state_t g_dispense_state = DISPENSE_IDLE;
static volatile float g_tank_level = 1.0f;
static volatile bool  g_drain_requested = false;
static volatile int   g_active_pump_gpio = -1;

// Water conservation tracker
static float g_total_water = -1.0f;

// =============================================================================
// Helpers
// =============================================================================

static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
static inline float randf(void) {
    return (float)(esp_random() & 0xFFFF) / 65536.0f;
}
static inline float randf_range(float lo, float hi) {
    return lo + randf() * (hi - lo);
}

// =============================================================================
// LED strip
// =============================================================================

static led_strip_handle_t led_strip = NULL;

static void led_init(void) {
    led_strip_config_t strip_cfg = {};
    strip_cfg.strip_gpio_num   = LED_GPIO;
    strip_cfg.max_leds         = NUM_LEDS;
    strip_cfg.led_model        = LED_MODEL_WS2812;
    strip_cfg.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;
    strip_cfg.flags.invert_out = false;

    led_strip_rmt_config_t rmt_cfg = {};
    rmt_cfg.clk_src           = RMT_CLK_SRC_DEFAULT;
    rmt_cfg.resolution_hz     = 10 * 1000 * 1000;
    rmt_cfg.mem_block_symbols = 64;
    rmt_cfg.flags.with_dma    = false;

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &led_strip));
    led_strip_clear(led_strip);
    led_strip_refresh(led_strip);
}

static int xy_to_index(int col, int row) {
    if (col < 0 || col >= MATRIX_COLS || row < 0 || row >= MATRIX_ROWS) return -1;
    row = MATRIX_ROWS - 1 - row;
    if (SERPENTINE && (row & 1))
        return row * MATRIX_COLS + (MATRIX_COLS - 1 - col);
    return row * MATRIX_COLS + col;
}

// Framebuffer
static uint8_t fb_r[MATRIX_COLS][MATRIX_ROWS];
static uint8_t fb_g[MATRIX_COLS][MATRIX_ROWS];
static uint8_t fb_b[MATRIX_COLS][MATRIX_ROWS];

static void fb_clear(void) {
    memset(fb_r, 0, sizeof(fb_r));
    memset(fb_g, 0, sizeof(fb_g));
    memset(fb_b, 0, sizeof(fb_b));
}

static void fb_set(int col, int row, uint8_t r, uint8_t g, uint8_t b) {
    if (col < 0 || col >= MATRIX_COLS || row < 0 || row >= MATRIX_ROWS) return;
    fb_r[col][row] = r; fb_g[col][row] = g; fb_b[col][row] = b;
}

static void fb_add(int col, int row, uint8_t r, uint8_t g, uint8_t b) {
    if (col < 0 || col >= MATRIX_COLS || row < 0 || row >= MATRIX_ROWS) return;
    int nr = fb_r[col][row] + r; if (nr > 255) nr = 255;
    int ng = fb_g[col][row] + g; if (ng > 255) ng = 255;
    int nb = fb_b[col][row] + b; if (nb > 255) nb = 255;
    fb_r[col][row] = (uint8_t)nr; fb_g[col][row] = (uint8_t)ng; fb_b[col][row] = (uint8_t)nb;
}

static void fb_flush(void) {
    for (int col = 0; col < MATRIX_COLS; col++) {
        for (int row = 0; row < MATRIX_ROWS; row++) {
            int idx = xy_to_index(col, row);
            if (idx < 0 || idx >= NUM_LEDS) continue;
            uint8_t r = (uint8_t)(((uint16_t)fb_r[col][row] * BRIGHTNESS) >> 8);
            uint8_t g = (uint8_t)(((uint16_t)fb_g[col][row] * BRIGHTNESS) >> 8);
            uint8_t b = (uint8_t)(((uint16_t)fb_b[col][row] * BRIGHTNESS) >> 8);
            led_strip_set_pixel(led_strip, idx, r, g, b);
        }
    }
}

// =============================================================================
// Shallow Water Simulation (with conservation + adaptive damping)
// =============================================================================

static float h[MATRIX_COLS];
static float u[MATRIX_COLS + 1];

static void sim_init(float fill) {
    float base_height = fill * MATRIX_ROWS;
    for (int i = 0; i < MATRIX_COLS; i++) h[i] = base_height;
    memset(u, 0, sizeof(u));
    g_total_water = base_height * MATRIX_COLS;
}

static void sim_update(float accel_horizontal) {
    float force = accel_horizontal * ACCEL_SCALE;

    // Adaptive damping: heavier friction when water level is low
    float avg_h = g_total_water / MATRIX_COLS;
    float fill_ratio = clampf(avg_h / MATRIX_ROWS, 0.05f, 1.0f);
    float effective_damping = DAMPING * fill_ratio + (1.0f - fill_ratio) * 0.95f;

    for (int i = 1; i < MATRIX_COLS; i++) {
        float dh = h[i - 1] - h[i];
        u[i] += WAVE_SPEED * dh;
        u[i] += force;
        u[i] *= effective_damping;
    }
    u[0] = 0;
    u[MATRIX_COLS] = 0;
    for (int i = 0; i < MATRIX_COLS; i++) {
        float flow_in  = u[i];
        float flow_out = u[i + 1];
        float max_drain = h[i] * 0.25f;
        if (flow_out > 0) flow_out = fminf(flow_out, max_drain);
        if (flow_in  < 0) flow_in  = fmaxf(flow_in, -max_drain);
        h[i] += flow_in - flow_out;
        h[i] = clampf(h[i], 0.0f, (float)MATRIX_ROWS);
    }

    // Enforce water conservation — fix phantom water from clamping
    float current_total = 0;
    for (int i = 0; i < MATRIX_COLS; i++) current_total += h[i];
    if (g_total_water >= 0 && current_total > 0.01f) {
        float scale = g_total_water / current_total;
        for (int i = 0; i < MATRIX_COLS; i++) h[i] *= scale;
    }
}

static void sim_drain_toward(float target_fill) {
    float target_h = target_fill * MATRIX_ROWS;
    float current_total = 0;
    for (int i = 0; i < MATRIX_COLS; i++) current_total += h[i];
    float avg_current = current_total / MATRIX_COLS;

    if (avg_current <= target_h) return;

    float drain_rate = (avg_current - target_h) / (PUMP_RUN_TIME_MS / 16.0f);
    float scale = 1.0f - (drain_rate / fmaxf(avg_current, 0.01f));
    scale = clampf(scale, 0.95f, 1.0f);

    for (int i = 0; i < MATRIX_COLS; i++) {
        h[i] *= scale;
        if (h[i] < 0) h[i] = 0;
    }

    // Update conservation target to match new lower level
    g_total_water = 0;
    for (int i = 0; i < MATRIX_COLS; i++) g_total_water += h[i];
}

// =============================================================================
// Splash Particles (water-conserving)
// =============================================================================

typedef struct {
    float x, y, vx, vy, life;
    bool active;
} splash_particle_t;

static splash_particle_t particles[MAX_PARTICLES];

static void particles_init(void) { memset(particles, 0, sizeof(particles)); }

static void spawn_particle(int col, float launch_speed, float h_speed) {
    if (h[col] < DROPLET_WATER + 0.1f) return;
    for (int i = 0; i < MAX_PARTICLES; i++) {
        if (!particles[i].active) {
            float surface_y = h[col];
            h[col] -= DROPLET_WATER;
            if (h[col] < 0) h[col] = 0;
            particles[i].active = true;
            particles[i].x  = (float)col + randf_range(-0.3f, 0.3f);
            particles[i].y  = surface_y + 0.1f;
            particles[i].vx = h_speed + randf_range(-SPLASH_HSPEED, SPLASH_HSPEED);
            particles[i].vy = launch_speed * randf_range(0.7f, 1.0f);
            particles[i].life = 1.0f;
            return;
        }
    }
}

static void particles_update(float accel_x, float accel_y) {
    for (int i = 0; i < MAX_PARTICLES; i++) {
        if (!particles[i].active) continue;
        splash_particle_t *p = &particles[i];
        p->vy -= PARTICLE_GRAVITY;
        p->vx += accel_x * 0.002f;
        p->vx *= PARTICLE_DAMPING;
        p->vy *= PARTICLE_DAMPING;
        p->x += p->vx;
        p->y += p->vy;
        p->life -= 0.005f;
        if (p->x < 0) { p->x = -p->x; p->vx = -p->vx * 0.5f; }
        if (p->x > MATRIX_COLS - 1) {
            p->x = 2 * (MATRIX_COLS - 1) - p->x;
            p->vx = -p->vx * 0.5f;
        }
        int col = (int)(p->x + 0.5f);
        col = col < 0 ? 0 : (col >= MATRIX_COLS ? MATRIX_COLS - 1 : col);
        float surface = h[col];
        if (p->y <= surface && p->vy <= 0) {
            h[col] += DROPLET_WATER;
            if (col > 0 && col < MATRIX_COLS - 1) {
                h[col]     -= 0.02f;
                h[col - 1] += 0.01f;
                h[col + 1] += 0.01f;
            }
            p->active = false;
            continue;
        }
        if (p->y < -1.0f || p->y > MATRIX_ROWS + 2 || p->life <= 0) {
            int return_col = (int)(p->x + 0.5f);
            return_col = return_col < 0 ? 0 : (return_col >= MATRIX_COLS ? MATRIX_COLS - 1 : return_col);
            h[return_col] += DROPLET_WATER;
            p->active = false;
        }
    }
}

static void particles_spawn(float accel_mag, float accel_horizontal) {
    if (accel_mag < SPLASH_THRESHOLD) return;
    float intensity = clampf((accel_mag - SPLASH_THRESHOLD) /
                             (SPLASH_HEAVY - SPLASH_THRESHOLD), 0.0f, 1.0f);
    int count = (int)(intensity * SPAWN_RATE) + 1;
    for (int s = 0; s < count; s++) {
        int col = (int)(randf() * MATRIX_COLS);
        if (col >= MATRIX_COLS) col = MATRIX_COLS - 1;
        if (h[col] < 0.5f) continue;
        float launch = randf_range(SPLASH_SPEED_MIN, SPLASH_SPEED_MAX) * (0.5f + intensity);
        float h_speed = accel_horizontal * 0.01f;
        spawn_particle(col, launch, h_speed);
    }
}

// =============================================================================
// Rendering
// =============================================================================

static void render(void) {
    fb_clear();
    for (int col = 0; col < MATRIX_COLS; col++) {
        float water_h = h[col];
        int surface_row = (int)water_h;
        float frac = water_h - (float)surface_row;
        for (int row = 0; row < MATRIX_ROWS; row++) {
            uint8_t r, g, b;
            if (row < surface_row) {
                float depth_ratio = (float)row / fmaxf(1.0f, water_h);
                r = (uint8_t)(depth_ratio * 10);
                g = (uint8_t)(10 + depth_ratio * 60);
                b = (uint8_t)(40 + depth_ratio * 160);
            } else if (row == surface_row) {
                r = (uint8_t)(180 * frac);
                g = (uint8_t)(220 * frac);
                b = (uint8_t)(255 * frac);
            } else if (row <= surface_row + SURFACE_GLOW) {
                float dist = (float)(row - surface_row);
                float falloff = fmaxf(0.0f, 1.0f - dist / (SURFACE_GLOW + 1.0f));
                falloff *= frac;
                r = (uint8_t)(40 * falloff);
                g = (uint8_t)(80 * falloff);
                b = (uint8_t)(120 * falloff);
            } else {
                r = 0; g = 0; b = 1;
            }
            fb_set(col, row, r, g, b);
        }
    }
    for (int i = 0; i < MAX_PARTICLES; i++) {
        if (!particles[i].active) continue;
        const splash_particle_t *p = &particles[i];
        int px = (int)(p->x + 0.5f);
        int py = (int)(p->y + 0.5f);
        float brightness = clampf(p->life, 0.0f, 1.0f);
        uint8_t dr = (uint8_t)(100 * brightness);
        uint8_t dg = (uint8_t)(130 * brightness);
        uint8_t db = (uint8_t)(160 * brightness);
        fb_add(px, py, dr, dg, db);
        if (fabsf(p->vy) > 0.1f || fabsf(p->vx) > 0.1f) {
            int tx = px - (p->vx > 0 ? 1 : (p->vx < 0 ? -1 : 0));
            int ty = py - (p->vy > 0 ? 1 : (p->vy < 0 ? -1 : 0));
            fb_add(tx, ty, (uint8_t)(dr * DROPLET_TRAIL), (uint8_t)(dg * DROPLET_TRAIL), (uint8_t)(db * DROPLET_TRAIL));
        }
        uint8_t gr = (uint8_t)(dr * 0.2f);
        uint8_t gg = (uint8_t)(dg * 0.2f);
        uint8_t gb = (uint8_t)(db * 0.2f);
        fb_add(px - 1, py, gr, gg, gb);
        fb_add(px + 1, py, gr, gg, gb);
        fb_add(px, py - 1, gr, gg, gb);
        fb_add(px, py + 1, gr, gg, gb);
    }
    fb_flush();
}

// =============================================================================
// Startup animation
// =============================================================================

static void startup_animation(void) {
    float target = 1.0f * MATRIX_ROWS;
    for (int fill_row = 0; fill_row <= (int)target; fill_row++) {
        fb_clear();
        for (int col = 0; col < MATRIX_COLS; col++) {
            for (int row = 0; row <= fill_row; row++) {
                float depth_ratio = (float)row / fmaxf(1.0f, target);
                fb_set(col, row,
                    (uint8_t)(depth_ratio * 10),
                    (uint8_t)(10 + depth_ratio * 60),
                    (uint8_t)(40 + depth_ratio * 160));
            }
        }
        fb_flush();
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelay(pdMS_TO_TICKS(500));
}

static void error_blink_red(void) {
    while (1) {
        for (int i = 0; i < NUM_LEDS; i++)
            led_strip_set_pixel(led_strip, i, 255, 0, 0);
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(500));
        led_strip_clear(led_strip);
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// =============================================================================
// BNO055 init with retry
// =============================================================================

static bno055_t imu = {};

static void bno055_init_with_retry(i2c_master_dev_handle_t dev_handle) {
    imu.config.slave_handle = dev_handle;
    imu.config.reset.method = RESET_SW;

    ESP_LOGI(TAG, "Waiting for BNO055...");
    for (int attempt = 1; attempt <= BNO055_INIT_RETRIES; attempt++) {
        for (int i = 0; i < NUM_LEDS; i++)
            led_strip_set_pixel(led_strip, i, 0, 0, 30);
        led_strip_refresh(led_strip);

        esp_err_t ret = bno055_initialize(&imu);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "BNO055 detected on attempt %d!", attempt);
            led_strip_clear(led_strip);
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(200));
            return;
        }
        ESP_LOGW(TAG, "BNO055 init attempt %d/%d failed (0x%x)", attempt, BNO055_INIT_RETRIES, ret);
        led_strip_clear(led_strip);
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(BNO055_RETRY_DELAY_MS));
    }
    ESP_LOGE(TAG, "BNO055 not found! Check wiring.");
    error_blink_red();
}

// =============================================================================
// Task 1: Fluid Simulation
// =============================================================================

static void fluid_task(void *arg) {
    int64_t last_frame = esp_timer_get_time();
    int64_t last_print = last_frame;
    float drain_target = -1.0f;

    while (1) {
        int64_t now = esp_timer_get_time();
        float dt = (now - last_frame) / 1000000.0f;
        if (dt < (1.0f / SIM_FPS)) { vTaskDelay(1); continue; }
        if (dt > 0.05f) dt = 0.05f;
        last_frame = now;

        if (g_drain_requested) {
            g_drain_requested = false;
            drain_target = g_tank_level;
            ESP_LOGI(TAG, "Draining to %.0f%%", drain_target * 100.0f);
        }

        if (drain_target >= 0.0f) {
            sim_drain_toward(drain_target);
            float total = 0;
            for (int i = 0; i < MATRIX_COLS; i++) total += h[i];
            float avg = total / MATRIX_COLS;
            if (avg <= drain_target * MATRIX_ROWS + 0.05f) {
                drain_target = -1.0f;
            }
        }

        esp_err_t err = bno055_get_readings(&imu, ACCELEROMETER);
        if (err != ESP_OK) { continue; }

        float ax = imu.raw_acceleration.x;
        float ay = imu.raw_acceleration.y;
        float az = imu.raw_acceleration.z;
        float accel_horizontal = ax;
        float accel_mag = sqrtf(ax * ax + ay * ay + az * az);

        sim_update(accel_horizontal);
        particles_spawn(accel_mag, accel_horizontal);
        particles_update(accel_horizontal, ax);
        render();
        led_strip_refresh(led_strip);

        if (now - last_print > 1000000) {
            last_print = now;

            const char *state_str = "IDLE";
            switch (g_dispense_state) {
                case DISPENSE_CUP_DETECTED: state_str = "CUP DETECTED"; break;
                case DISPENSE_PUMPING:      state_str = "PUMPING";      break;
                case DISPENSE_COOLDOWN:     state_str = "COOLDOWN";     break;
                default: break;
            }

            ESP_LOGI(TAG, "Tank: %.0f%% | ToF: %s%d mm | State: %s | Pump GPIO: %d",
                g_tank_level * 100.0f,
                g_tof_valid ? "" : "-- ",
                g_tof_valid ? (int)g_tof_distance_mm : 0,
                state_str,
                (int)g_active_pump_gpio);
        }
    }
}

// =============================================================================
// Task 2: ToF + Dispense State Machine (with drink selection)
// =============================================================================

static void tof_task(void *arg) {
    VL53L0X sensor;
    sensor.setTimeout(500);

    ESP_LOGI(TAG, "Waiting for VL53L0X...");
    bool initialized = false;
    for (int attempt = 1; attempt <= 10; attempt++) {
        if (sensor.init()) {
            initialized = true;
            ESP_LOGI(TAG, "VL53L0X initialized on attempt %d!", attempt);
            break;
        }
        ESP_LOGW(TAG, "VL53L0X init attempt %d/10 failed", attempt);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    if (!initialized) {
        ESP_LOGE(TAG, "VL53L0X not found! Dispense disabled.");
        vTaskDelete(NULL);
        return;
    }

    // Init pump outputs
    gpio_set_direction((gpio_num_t)PUMP_A_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)PUMP_B_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)PUMP_C_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)PUMP_A_GPIO, 0);
    gpio_set_level((gpio_num_t)PUMP_B_GPIO, 0);
    gpio_set_level((gpio_num_t)PUMP_C_GPIO, 0);

    // Init selection inputs with pull-down
    gpio_set_direction((gpio_num_t)SELECT_GPIO_1, GPIO_MODE_INPUT);
    gpio_set_direction((gpio_num_t)SELECT_GPIO_2, GPIO_MODE_INPUT);
    gpio_pulldown_en((gpio_num_t)SELECT_GPIO_1);
    gpio_pulldown_en((gpio_num_t)SELECT_GPIO_2);

    int64_t cup_first_seen = 0;
    int64_t pump_start_time = 0;
    int64_t cooldown_start = 0;

    while (1) {
        uint16_t distance_mm = sensor.readRangeSingleMillimeters();
        bool valid = !sensor.timeoutOccurred() && distance_mm < 8190;

        if (valid) {
            g_tof_distance_mm = distance_mm;
            g_tof_valid = true;
        }

        bool cup_present = valid && (distance_mm < CUP_DETECT_DIST_MM);
        int64_t now = esp_timer_get_time() / 1000;

        switch (g_dispense_state) {

            case DISPENSE_IDLE:
                if (cup_present && g_tank_level > 0.01f) {
                    cup_first_seen = now;
                    g_dispense_state = DISPENSE_CUP_DETECTED;
                    ESP_LOGI(TAG, "Cup detected! Hold for %d seconds...", CUP_HOLD_TIME_MS / 1000);
                }
                break;

            case DISPENSE_CUP_DETECTED:
                if (!cup_present) {
                    g_dispense_state = DISPENSE_IDLE;
                    ESP_LOGI(TAG, "Cup removed, resetting.");
                } else if ((now - cup_first_seen) >= CUP_HOLD_TIME_MS) {
                    if (g_tank_level <= 0.01f) {
                        ESP_LOGW(TAG, "Tank empty! Cannot dispense.");
                        g_dispense_state = DISPENSE_IDLE;
                    } else {
                        // Read drink selection pins
                        int sel1 = gpio_get_level((gpio_num_t)SELECT_GPIO_1);
                        int sel2 = gpio_get_level((gpio_num_t)SELECT_GPIO_2);

                        int pump_gpio = -1;
                        const char *drink_name = "NONE";

                        if (sel2 == 1 && sel1 == 0) {
                            pump_gpio = PUMP_A_GPIO;
                            drink_name = "A";
                        } else if (sel2 == 0 && sel1 == 1) {
                            pump_gpio = PUMP_B_GPIO;
                            drink_name = "B";
                        } else if (sel2 == 1 && sel1 == 1) {
                            pump_gpio = PUMP_C_GPIO;
                            drink_name = "C";
                        }

                        if (pump_gpio < 0) {
                            ESP_LOGW(TAG, "No drink selected (sel1=%d sel2=%d), ignoring.", sel1, sel2);
                            g_dispense_state = DISPENSE_IDLE;
                        } else {
                            g_dispense_state = DISPENSE_PUMPING;
                            pump_start_time = now;
                            g_active_pump_gpio = pump_gpio;

                            gpio_set_level((gpio_num_t)pump_gpio, 1);

                            float new_level = g_tank_level - (DISPENSE_PERCENT / 100.0f);
                            if (new_level < 0) new_level = 0;
                            g_tank_level = new_level;
                            g_drain_requested = true;

                            ESP_LOGI(TAG, "DISPENSING drink %s! Pump GPIO %d ON. Tank -> %.0f%%",
                                     drink_name, pump_gpio, g_tank_level * 100.0f);
                        }
                    }
                }
                break;

            case DISPENSE_PUMPING:
                if ((now - pump_start_time) >= PUMP_RUN_TIME_MS) {
                if (g_active_pump_gpio >= 0) {
                    gpio_set_level((gpio_num_t)g_active_pump_gpio, 0);
                }
                g_active_pump_gpio = -1;

                // Pulse GPIO 0 high for 1 second
                gpio_set_level((gpio_num_t)0, 1);
                vTaskDelay(pdMS_TO_TICKS(1000));
                gpio_set_level((gpio_num_t)0, 0);

                cooldown_start = esp_timer_get_time() / 1000;  // refresh timestamp after delay
                g_dispense_state = DISPENSE_COOLDOWN;
                ESP_LOGI(TAG, "Pump OFF. GPIO 0 pulsed. Cooldown...");
                }
                break;


            case DISPENSE_COOLDOWN:
                if ((now - cooldown_start) >= COOLDOWN_TIME_MS) {
                    g_dispense_state = DISPENSE_IDLE;
                    ESP_LOGI(TAG, "Ready for next cup.");
                }
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// =============================================================================
// app_main
// =============================================================================

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "=== Drink Dispenser Starting ===");

    led_init();

    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port   = I2C_NUM_0;
    bus_cfg.sda_io_num = (gpio_num_t)I2C_SDA_GPIO;
    bus_cfg.scl_io_num = (gpio_num_t)I2C_SCL_GPIO;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = true;

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

    i2c_device_config_t bno_dev_cfg = {};
    bno_dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    bno_dev_cfg.device_address  = BNO055_ADDR;
    bno_dev_cfg.scl_speed_hz    = 400000;
    bno_dev_cfg.scl_wait_us     = 0xFFFF;

    i2c_master_dev_handle_t bno_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &bno_dev_cfg, &bno_handle));

    i2c.init(bus_handle, VL53L0X_ADDR);

    bno055_init_with_retry(bno_handle);
    esp_err_t ret = bno055_configure(&imu, NDOF_MODE, ACC_M_S2 | GY_DPS | EUL_DEG | TEMP_C);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BNO055 configure failed!");
        error_blink_red();
    }
    vTaskDelay(pdMS_TO_TICKS(1000));

    startup_animation();

    sim_init(1.0f);
    particles_init();

    xTaskCreate(fluid_task, "fluid", 8192, NULL, 5, NULL);
    xTaskCreate(tof_task,   "tof",   4096, NULL, 4, NULL);

    ESP_LOGI(TAG, "Dispenser ready! Tank at 100%%");
    ESP_LOGI(TAG, "Hold cup within %d mm for %d seconds to dispense.", CUP_DETECT_DIST_MM, CUP_HOLD_TIME_MS / 1000);
    ESP_LOGI(TAG, "Drink selection: GPIO2=0,GPIO1=1 -> A (GPIO%d) | GPIO2=1,GPIO1=0 -> B (GPIO%d) | GPIO2=1,GPIO1=1 -> C (GPIO%d)",
             PUMP_A_GPIO, PUMP_B_GPIO, PUMP_C_GPIO);
}