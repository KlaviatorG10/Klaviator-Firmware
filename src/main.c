/*
 * KDAA Firmware - The Deterministic Executor (v3.0)
 * Utviklet av: Yousef (Tech Lead)
 * Operativsystem: Zephyr RTOS
 * Maskinvare: nRF54L15
 * 
 * HVORFOR: Denne koden er hjertet i den deterministiske utførelsen. 
 * Den eliminerer OS-jitter ved å bruke brikkens interne maskinvare-klokke.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>

#define UART_DEVICE_NODE DT_CHOSEN(zephyr_console)

/* Solenoid definition from Device Tree alias */
#define SOLENOID_NODE DT_ALIAS(solenoid)
static const struct gpio_dt_spec solenoid = GPIO_DT_SPEC_GET(SOLENOID_NODE, gpios);

#define MSG_SIZE 64
#define BUFFER_SIZE 256 // Plass til 256 planlagte hendelser i køen

/* --- DATASTRUKTURER FOR DETERMINISTISK UTFØRELSE --- */

typedef enum { EVENT_MOVE, EVENT_STRIKE } event_type_t;

struct kdaa_event {
    uint32_t t_target;    // Måltidspunkt i ms fra SYNC
    event_type_t type;    // MOVE eller STRIKE
    char hand;            // 'L' eller 'R'
    int val1;             // Posisjon (for MOVE) eller Note (for STRIKE)
    int val2;             // Velocity (kun for STRIKE)
    bool active;          // Er denne slotten i bruk?
};

static struct kdaa_event event_buffer[BUFFER_SIZE];
static uint32_t start_time_ms = 0;
static bool is_running = false;

/* --- UART OG MELDINGSHÅNDTERING --- */

K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 32, 4);
static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

/* --- SEQUENCER TRÅD (Høyprioritets-utfører) --- */

#define STACK_SIZE 1024
#define PRIORITY 1 // Høy prioritet for å sikre presis timing
K_THREAD_STACK_DEFINE(seq_stack, STACK_SIZE);
struct k_thread seq_thread_data;

void sequencer_loop(void *unused1, void *unused2, void *unused3) {
    while (1) {
        if (is_running) {
            uint32_t now = k_uptime_get_32() - start_time_ms;

            for (int i = 0; i < BUFFER_SIZE; i++) {
                if (event_buffer[i].active && now >= event_buffer[i].t_target) {
                    
                    /* UTFØR HANDLING */
                    if (event_buffer[i].type == EVENT_MOVE) {
                        // HER KOMMER MOTOR-LOGIKK FRA PARTNER
                        printk("[EXEC] MOVE Hand:%c Pos:%d @ T:%u\n", 
                               event_buffer[i].hand, event_buffer[i].val1, now);
                    } else {
                        // UTFØR STRIKE MED SOLENOID
                        if (event_buffer[i].val2 > 0) {
                            gpio_pin_set_dt(&solenoid, 1);
                        } else {
                            gpio_pin_set_dt(&solenoid, 0);
                        }
                        
                        printk("[EXEC] STRIKE Hand:%c Note:%d Vel:%d @ T:%u\n", 
                               event_buffer[i].hand, event_buffer[i].val1, event_buffer[i].val2, now);
                    }
                    
                    event_buffer[i].active = false; // Frigjør slotten
                }
            }
        }
        k_usleep(500); // Sjekk hver 0.5ms for ekstrem presisjon
    }
}

/* --- PARSER OG KOMMANDO-HÅNDTERING --- */

void parse_kdaa_cmd(char *msg) {
    /* 
     * HVORFOR: Lynrask parsing av den tidsstemplede protokollen.
     * Vi bruker sscanf for å hente ut verdier fra Hub-formatet.
     */
    
    // SYNC:T - Nullstill klokken
    if (strncmp(msg, "SYNC:", 5) == 0) {
        start_time_ms = k_uptime_get_32();
        is_running = true;
        printk("[MCU] SYNC OK - Clock Reset\n");
        return;
    }

    // STOP - Nødstopp
    if (strcmp(msg, "STOP") == 0) {
        is_running = false;
        for(int i=0; i<BUFFER_SIZE; i++) event_buffer[i].active = false;
        gpio_pin_set_dt(&solenoid, 0); // Sikkerhet: skru av solenoid
        printk("[MCU] HALTED\n");
        return;
    }

    // M:H:P:T - MOVE kommando
    if (msg[0] == 'M' && msg[1] == ':') {
        char h; int p; uint32_t t;
        if (sscanf(msg, "M:%c:%d:%u", &h, &p, &t) == 3) {
            for (int i = 0; i < BUFFER_SIZE; i++) {
                if (!event_buffer[i].active) {
                    event_buffer[i].t_target = t;
                    event_buffer[i].type = EVENT_MOVE;
                    event_buffer[i].hand = h;
                    event_buffer[i].val1 = p;
                    event_buffer[i].active = true;
                    return;
                }
            }
        }
    }

    // S:H:N:V:T - STRIKE kommando
    if (msg[0] == 'S' && msg[1] == ':') {
        char h; int n, v; uint32_t t;
        if (sscanf(msg, "S:%c:%d:%d:%u", &h, &n, &v, &t) == 4) {
            for (int i = 0; i < BUFFER_SIZE; i++) {
                if (!event_buffer[i].active) {
                    event_buffer[i].t_target = t;
                    event_buffer[i].type = EVENT_STRIKE;
                    event_buffer[i].hand = h;
                    event_buffer[i].val1 = n;
                    event_buffer[i].val2 = v;
                    event_buffer[i].active = true;
                    return;
                }
            }
        }
    }
}

/* --- UART ISR OG MAIN --- */

void uart_cb(const struct device *dev, void *user_data) {
    uint8_t c;
    if (!uart_irq_update(uart_dev) || !uart_irq_rx_ready(uart_dev)) return;
    while (uart_fifo_read(uart_dev, &c, 1) == 1) {
        if (c == '\n' || c == '\r') {
            if (rx_buf_pos > 0) {
                rx_buf[rx_buf_pos] = '\0';
                k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);
                rx_buf_pos = 0;
            }
        } else if (rx_buf_pos < MSG_SIZE - 1) {
            rx_buf[rx_buf_pos++] = c;
        }
    }
}

int main(void) {
    /* Test visual boot indicator (LED0) */
    const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
    if (gpio_is_ready_dt(&led)) {
        gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
        k_msleep(500); /* LED is ON for 0.5s during boot */
        gpio_pin_set_dt(&led, 0);
    }

    k_msleep(1000); /* Wait for terminal to connect over USB */
    printk("\n--- KDAA Firmware Booting ---\n");

    if (!device_is_ready(uart_dev)) {
        printk("Error: UART device not ready\n");
        return 0;
    }

    /* Initialize the solenoid GPIO */
    if (!gpio_is_ready_dt(&solenoid)) {
        printk("Error: Solenoid device not ready\n");
        return 0;
    }
    gpio_pin_configure_dt(&solenoid, GPIO_OUTPUT_INACTIVE);

    uart_irq_callback_user_data_set(uart_dev, uart_cb, NULL);
    uart_irq_rx_enable(uart_dev);

    /* Start Sequencer Tråden */
    k_thread_create(&seq_thread_data, seq_stack, K_THREAD_STACK_SIZEOF(seq_stack),
                    sequencer_loop, NULL, NULL, NULL, PRIORITY, 0, K_NO_WAIT);

    printk("[MCU] KDAA Executor Ready @ 1M Baud\n");

    while (1) {
        char cmd[MSG_SIZE];
        if (k_msgq_get(&uart_msgq, &cmd, K_FOREVER) == 0) {
            parse_kdaa_cmd(cmd);
        }
    }
    return 0;
}
