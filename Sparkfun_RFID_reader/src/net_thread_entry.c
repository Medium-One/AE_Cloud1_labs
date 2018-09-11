/*
 *  Copyright (c) 2017 Medium One, Inc
 *  www.mediumone.com
 *
 *  Portions of this work may be based on third party contributions.
 *  Medium One, Inc reserves copyrights to this work whose
 *  license terms are defined under a separate Software License
 *  Agreement (SLA).  Re-distribution of any or all of this work,
 *  in source or binary form, is prohibited unless authorized by
 *  Medium One, Inc under SLA.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 *  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 *  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 *  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "net_thread.h"
#include "system_thread.h"
#include "r_flash_api.h"
#include "nxd_dns.h"
#include "nxd_dhcp_client.h"
#include "fx_stdio.h"
#include "led.h"
#include "agg.h"
#include "app.h"
#include "file_transfer.h"
#include "helpers.h"

#include <m1_agent.h>
#ifdef USE_M1DIAG
#include <m1diagnostics_agent.h>
#endif


//************ macros ************
#define TLS_PACKET_BUFFER_MEM_SIZE (6 * 1024)
#define MQTT_STACK_SIZE (8 * 1024)
#define MAX_MQTT_MSGS 10
#define MQTT_MSG_MEM_SIZE (MAX_MQTT_MSGS * NXD_MQTT_MAX_MESSAGE_LENGTH)
#define CRYPTO_METADATA_MEM_SIZE 2104
#define MAX_CERTS 3
#define CERT_SIZE (2 * 1024)
#define CERT_MEM_SIZE (MAX_CERTS * CERT_SIZE)
#define SENSORS 13


// added by Michael Li
#define  RFID_ID_LEN        10  // only ID ascii character
extern char rfid_string[RFID_ID_LEN+1];  // including null at the end.

#ifdef USE_M1DIAG
#define DEV_DIAG

#ifdef USE_TLS
// old diag agent api takes single memory block for both packet buffers and metadata
#define DIAG_SSL_MEM_SIZE (TLS_PACKET_BUFFER_MEM_SIZE + CRYPTO_METADATA_MEM_SIZE)
#endif
#endif


//************ externs ************
extern char g_link_code[7];
extern TX_THREAD net_thread;
extern TX_THREAD adc_thread;
extern TX_THREAD bmc_thread;
extern TX_THREAD ens_thread;
extern TX_THREAD ms_thread;
extern TX_THREAD system_thread;
extern NX_IP g_http_ip;
extern NX_DNS g_dns_client;
extern const NX_SECURE_TLS_CRYPTO nx_crypto_tls_ciphers_synergys7;
extern FX_MEDIA g_qspi_media;

// added by Michael Li
extern TX_THREAD rfid_thread;

#ifdef USE_M1DIAG
/*
 * the diagnostics agent asynchronously samples peripherals.
 * to ensure it sees real-time updated data, we expose the "local" aggregate
 * structures used by the sensor threads
 */
extern volatile agg_t x;
extern volatile agg_t y;
extern volatile agg_t z;
extern volatile agg_t temp1;
extern volatile agg_t x_mag;
extern volatile agg_t y_mag;
extern volatile agg_t z_mag;
extern volatile agg_t temp2;
extern volatile agg_t humidity;
extern volatile agg_t temp3;
extern volatile agg_t pressure;
extern volatile agg_t mic;
extern volatile agg_t voice_fft;
#endif


//************ global variables ************
/*
 * global sensor aggregates
 *
 * sensor threads sample their sensors and aggregate in local aggregate
 * structures.
 * when a transfer is requested, the sensor thread copies the local aggregate
 * data into the corresponding global structure below.
 */
volatile agg_t g_x = { .name = "x_accel", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_y = { .name = "y_accel", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_z = { .name = "z_accel", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_x_mag = { .name = "x_mag", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_y_mag = { .name = "y_mag", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_z_mag = { .name = "z_mag", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_temp1 = { .name = "temp1", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_temp2 = { .name = "temp2", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_temp3 = { .name = "temp3", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_humidity = { .name = "humidity", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_pressure = { .name = "pressure", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_mic = { .name = "mic", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_voice_fft = { .name = "voice_fft", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile uint32_t g_x_zero_crossings = 0, g_y_zero_crossings = 0, g_z_zero_crossings = 0;

app_config_t app_config;


//************ local variables ************
#ifdef USE_TLS
static char ssl_mem[TLS_PACKET_BUFFER_MEM_SIZE];
static char crypto_mem[CRYPTO_METADATA_MEM_SIZE];
static char cert_mem[CERT_MEM_SIZE];
static char ota_crypto_mem[CRYPTO_METADATA_MEM_SIZE];
#endif
static char mqtt_stack[MQTT_STACK_SIZE];
static char mqtt_msg_mem[MQTT_MSG_MEM_SIZE];

// TODO: use message framework to update instead of volatile?
static volatile ULONG sample_period = 15 * 60 * 100;

static volatile agg_t * const sensors[SENSORS] = {&g_x,
                                                  &g_y,
                                                  &g_z,
                                                  &g_x_mag,
                                                  &g_y_mag,
                                                  &g_z_mag,
                                                  &g_temp1,
                                                  &g_temp2,
                                                  &g_temp3,
                                                  &g_humidity,
                                                  &g_pressure,
                                                  &g_mic,
                                                  &g_voice_fft
};

// holds the bitmask of leds that should be blinked in the RTC callback
static volatile uint8_t leds_to_toggle = 0;
// holds the number of blinks that should currently be performed in the RTC callback
static volatile int n_blinks = 0;

// OTA parameters
static char g_update_url[640];
static uint32_t g_update_hash[8];
static volatile uint8_t g_update_firmware = 0;

// connection details for application to sandbox
static const char * rna_broker_url = "mqtt-renesas-na-sandbox.mediumone.com";

#ifdef USE_TLS
static const uint16_t rna_port = 61620;
#else
static const uint16_t rna_port = 61619;
#endif

#ifdef USE_M1DIAG
static periphery_access_t x_accel_access = {(char)1, NULL, {.address=&x.value}};
static periphery_access_t y_accel_access = {(char)1, NULL, {.address=&y.value}};
static periphery_access_t z_accel_access = {(char)1, NULL, {.address=&z.value}};
static periphery_access_t temp_access = {(char)1, NULL, {.address=&temp1.value}};
static periphery_access_t xmag_access = {(char)1, NULL, {.address=&x_mag.value}};
static periphery_access_t ymag_access = {(char)1, NULL, {.address=&y_mag.value}};
static periphery_access_t zmag_access = {(char)1, NULL, {.address=&z_mag.value}};
static periphery_access_t temp2_access = {(char)1, NULL, {.address=&temp2.value}};
static periphery_access_t humidity_access = {(char)1, NULL, {.address=&humidity.value}};
static periphery_access_t temp3_access = {(char)1, NULL, {.address=&temp3.value}};
static periphery_access_t pressure_access = {(char)1, NULL, {.address=&pressure.value}};
static periphery_access_t mic_access = {(char)1, NULL, {.address=&mic.value}};
static periphery_access_t voice_fft_access = {(char)1, NULL, {.address=&voice_fft.value}};
#endif


//************ global functions ************
// extern GPIO pin setting function
void set_pin(ioport_port_pin_t pin, ioport_level_t level);



/*
 * handles messages published to one of the topics subscribed to by the m1 agent.
 * we use the convention of the first character of the msg indicating a command.
 * multiple commands can be sent in one message, separated by ';'. ';' must not
 * be used within a command. Supported commands are:
 *      - S: set new sampling period
 *      - T: update sensor threshold
 *      - L: set, clear, or toggle LED(s)
 *      - B: blink LED(s)
 *      - G: set GPIO pin output level
 */
static void m1_callback(int type, char * topic, char * msg, int length) {
    SSP_PARAMETER_NOT_USED(type);
    SSP_PARAMETER_NOT_USED(topic);
    SSP_PARAMETER_NOT_USED(length);
    char * p = msg, * temp_str;
    int temp;
    int msglen;

    // TODO: hack...
    msg[length] = '\0';
    while (p != NULL) {
        temp_str = strchr(p, ';');
        if (temp_str != NULL)
            msglen = temp_str - p;
        else
            msglen = length - (p - msg);
        if (type == 1) {
            switch (p[0]) {
                case 'S': {
                    ULONG new_sample_period;
                    sscanf(&p[1], "%lu", &new_sample_period);
                    sample_period = new_sample_period;
                    tx_thread_wait_abort(&net_thread);
                    break;
                }
                case 'T': {
                    char name[30];
                    float threshold;
                    int threshold_type;
                    if (sscanf(&p[1], "%29[^:]:%d:%f", name, &threshold_type, &threshold) == 3) {
                        for (int i = 0; i < SENSORS; i++) {
                            if (!strcmp(sensors[i]->name, name)) {
                                sensors[i]->absolute_threshold = threshold_type ? 1 : 0;
                                sensors[i]->threshold = threshold;
                                break;
                            }
                        }
                        tx_event_flags_set(&g_sensor_event_flags, BMC_THRESHOLD_UPDATE | ADC_THRESHOLD_UPDATE | ENS_THRESHOLD_UPDATE | MS_THRESHOLD_UPDATE, TX_OR);
                    }
                    break;
                }
                case 'L': {
                    int val;
                    sscanf(&p[1], "%d", &temp);
                    temp_str = strchr(&p[1], ':');
                    // no strnchr...
                    if ((temp_str != NULL) && ((temp_str - p) < msglen)) {
                        if (sscanf(temp_str + 1, "%d", &val) == 1)
                            set_led((uint8_t)temp, (uint8_t)val);
                    } else
                        toggle_leds((temp >> 2) & 0x01, (temp >> 1) & 0x01, temp & 0x01);
                    break;
                }
                case 'B':
                    n_blinks = 0;
                    if (sscanf(&p[1], "%d:%d", &temp, &n_blinks) < 1)
                        break;
                    if (!temp)
                        break;
                    if (g_rtc0.p_api->open(g_rtc0.p_ctrl, g_rtc0.p_cfg) != SSP_SUCCESS)
                        break;
                    leds_to_toggle = (uint8_t)temp;
                    g_rtc0.p_api->periodicIrqRateSet(g_rtc0.p_ctrl, RTC_PERIODIC_IRQ_SELECT_1_SECOND);
                    g_rtc0.p_api->calendarCounterStart(g_rtc0.p_ctrl);
                    g_rtc0.p_api->irqEnable(g_rtc0.p_ctrl, RTC_EVENT_PERIODIC_IRQ);
                    break;
                            // aded by Michael
                case 'V':   // RFID ADD or DELETE or Verify pass (green blink once) or fail (red blink once)
                    /* - Green: 1
                    - Yellow: 2
                    - Green + Yellow: 3
                    - Red: 4
                    - Green + Red: 5
                    - Yellow + Red: 6
                    - Green + Yellow + Red: 7
                    */
                     n_blinks = 1;
                     if (sscanf(&p[1], "%d", &temp) < 1)
                         break;
                     if (temp > 1)
                         break;
                     if (temp != 1) {
                         temp = 4;  // red led (fail)
                         set_led(1, 0); // clear green led if on
                     } else {
                         temp = 1;  // green led (pass)
                         set_led(4, 0); // clear red led if on
                     }
                     if (g_rtc0.p_api->open(g_rtc0.p_ctrl, g_rtc0.p_cfg) != SSP_SUCCESS)
                         break;
                     leds_to_toggle = (uint8_t)temp;
                     g_rtc0.p_api->periodicIrqRateSet(g_rtc0.p_ctrl, RTC_PERIODIC_IRQ_SELECT_1_SECOND);
                     g_rtc0.p_api->calendarCounterStart(g_rtc0.p_ctrl);
                     g_rtc0.p_api->irqEnable(g_rtc0.p_ctrl, RTC_EVENT_PERIODIC_IRQ);
                     break;
                case 'E': {   //  enable or disable  E#:#  E<LED>:<State>    LED=3 (red), 1 (green),  State = 0 (off), 1 (on)
                    int level;
                    if (sscanf(&p[1], "%d:%d",&temp,&level) == 2){
                        if (temp == 4 || temp == 1)
                            set_led((uint8_t)temp, (uint8_t)level);
                    }
                    break;
                          }
                case 'G': {
                    int pin;
                    int level;
                    if (sscanf(&p[1], "%d:%d:%d", &temp, &pin, &level) == 3)
                        set_pin(pin + (temp << 8), level == 0 ? 0 : 1);
                    break;
                }
                case 'U': {
                    int i = 0;
                    int minor;
                    int ret = sscanf(&p[1], "%d:%d:%*[^:]:%n",
                                     &temp,
                                     &minor,
                                     &i);
                    if ((ret == 2) && (i < msglen)) {
                        strncpy(g_update_url, &p[i + 1], (size_t)(msglen - i));
                        for (int j = 0; j < 8; j++) {
                            p[i - (j * 8)] = '\0';
                            g_update_hash[7 - j] = (uint32_t)strtoull(&p[i - ((j + 1) * 8)], NULL, 16);
                        }
                        if ((temp != VERSION_MAJOR) || (minor != VERSION_MINOR)) {
                            g_update_firmware = 1;
                            tx_thread_wait_abort(&net_thread);
                        }
                    }
                    break;
		}
                default:
                    break;
            }
        } else if (type == 2) {
            switch (p[0]) {
                case 'U': {
                    int i = 0;
                    int minor;
                    int ret = sscanf(&p[1], "%d:%d:%*[^:]:%n",
                                     &temp,
                                     &minor,
                                     &i);
                    if ((ret == 2) && (i < msglen)) {
                        strncpy(g_update_url, &p[i + 1], (size_t)(msglen - i));
                        for (int j = 0; j < 8; j++) {
                            p[i - (j * 8)] = '\0';
                            g_update_hash[7 - j] = (uint32_t)strtoull(&p[i - ((j + 1) * 8)], NULL, 16);
                        }
                        if ((temp != VERSION_MAJOR) || (minor != VERSION_MINOR))
                            g_update_firmware = 1;
                    }
                    break;
                }
            }
        }
        p = strchr(p, ';');
        if (p != NULL)
            p++;
    }
}

/*
 * rtc callback. blinks LEDs if required
 */
void rtc_callback(rtc_callback_args_t * p_args) {
    static int count = 0;
    if (p_args->event == RTC_EVENT_PERIODIC_IRQ) {
        if (count++ < (n_blinks * 2)) {
            toggle_leds((leds_to_toggle >> 2) & 0x01, (leds_to_toggle >> 1) & 0x01, leds_to_toggle & 0x01);
        } else {
            count = 0;
            g_rtc0.p_api->close(g_rtc0.p_ctrl);
        }
    }
}


/*
 * helper function to determine how many dataflash blocks are spanned
 */
static void bytes_to_df_blocks(uint32_t bytes, flash_info_t * flash_info, uint32_t * write_blocks, uint32_t * erase_blocks) {
    if (!bytes) {
        *write_blocks = 0;
        *erase_blocks = 0;
        return;
    }
    *write_blocks = bytes / flash_info->data_flash.p_block_array[0].block_size_write + 1;
    *erase_blocks = bytes / flash_info->data_flash.p_block_array[0].block_size + 1;
}

static int write_struct_to_flash(void * obj, size_t size) {
    flash_info_t df_info;
    ssp_err_t status;
    uint32_t write_blocks, erase_blocks;

    status = g_flash0.p_api->open(g_flash0.p_ctrl, g_flash0.p_cfg);
    if (status)
        return -1;
    status = g_flash0.p_api->infoGet(g_flash0.p_ctrl, &df_info);
    if (status)
        return -5;
    // TODO: return erase blocks vs write blocks
    bytes_to_df_blocks(size, &df_info, &write_blocks, &erase_blocks);
    status = g_flash0.p_api->erase(g_flash0.p_ctrl, 0x40100000, erase_blocks);
    if (status)
        return -2;
    status = g_flash0.p_api->write(g_flash0.p_ctrl, (uint32_t)obj, 0x40100000, write_blocks * df_info.data_flash.p_block_array[0].block_size_write);
    if (status)
        return -3;
    status = g_flash0.p_api->close(g_flash0.p_ctrl);
    if (status)
        return -4;
    return 0;
}

static int write_app_config_to_flash(app_config_t * p_app_config) {
    char buf[sizeof(app_config_t) + 1];
    buf[0] = 'P';
    memcpy(&buf[1], p_app_config, sizeof(app_config_t));
    return write_struct_to_flash(buf, sizeof(buf));
}

static void toggle_green_led(int disk, int network) {
    if (disk)
        set_led(2, 1);
    else if (network)
        set_led(1, 1);
    else
        set_led(3, 0);
}

static int read_certs_and_connect(const char * p_mqtt_broker_host,
                                  int mqtt_broker_port,
                                  project_credentials_t * project,
                                  user_credentials_t * registration,
                                  user_credentials_t * device) {
#ifdef CERT_BASED_AUTH
    char cert_buf[MAX_CERT_LENGTH];
    char key_buf[MAX_KEY_LENGTH];
#endif

    m1_connect_params params = {
                                .mqtt_url = p_mqtt_broker_host,
                                .mqtt_port = mqtt_broker_port,
                                .project = project,
                                .registration = registration,
                                .device = device,
                                .device_id = "s5d9",
                                .retry_delay = 5,
                                .retry_limit = 5,
                                .mqtt_heart_beat = 600,
                                .stack_mem = mqtt_stack,
                                .stack_mem_size = MQTT_STACK_SIZE,
                                .mqtt_msg_mem = mqtt_msg_mem,
                                .mqtt_msg_mem_size = MQTT_MSG_MEM_SIZE,
#ifdef USE_TLS
                                .tls_enabled = 1,
                                .ssl_mem = ssl_mem,
                                .ssl_mem_size = sizeof(ssl_mem),
                                .crypto_mem = crypto_mem,
                                .crypto_mem_size = sizeof(crypto_mem),
                                .cert_mem = cert_mem,
                                .cert_mem_size = CERT_MEM_SIZE,
#else
                                .tls_enabled = 0,
#endif
                                .p_ppool = &g_http_packet_pool,
                                .p_ip = &g_http_ip,
                                .p_dns = &g_dns_client
    };
#ifdef CERT_BASED_AUTH
#if 0
    FILE * device_cert = fopen("device.crt", "r");
    FILE * device_key = fopen("device.key", "r");
    if (device_cert && device_key) {
        memset(cert_buf, 0, sizeof(cert_buf));
        memset(key_buf, 0, sizeof(key_buf));
        fread(cert_buf, MAX_CERT_LENGTH, 1, device_cert);
        fread(key_buf, MAX_KEY_LENGTH, 1, device_key);
        fclose(device_cert);
        fclose(device_key);
        params.device_cert = cert_buf;
        params.device_key = key_buf;
    }
#endif
#endif

    return m1_auto_enroll_connect_2(&params);
}

static int device_compare(user_credentials_t * config1, user_credentials_t * config2) {
    if (strncmp(config1->password, config2->password, sizeof(config1->password)))
        return -1;
    if (strncmp(config1->user_id, config2->user_id, sizeof(config1->user_id)))
        return -1;
    return 0;
}

static int project_compare(project_credentials_t * config1, project_credentials_t * config2) {
    if (strncmp(config1->apikey, config2->apikey, sizeof(config1->apikey)))
        return -1;
    if (strncmp(config1->proj_id, config2->proj_id, sizeof(config1->proj_id)))
        return -1;
    return 0;
}

static int config_compare(app_config_t * config1, app_config_t * config2) {
    if (device_compare(&config1->device, &config2->device))
        return -1;
#ifdef M1_DIAG
    if (device_compare(&config1->diagnostics, &config2->diagnostics))
        return -2;
#endif
    if (device_compare(&config1->registration, &config2->registration))
        return -3;
    if (project_compare(&config1->project, &config2->project))
        return -4;
    return 0;
}

/*
 * extracts credentials from m1config.txt, and optionally m1user.txt and
 * m1broker.txt.
 *
 * if m1 diagnostics agent is enabled, checks dataflash for
 * auto-enrolled credentials. if credentials are present in dataflash, attempts
 * connection with those credentials. if connection fails, attempts connection
 * with registration user credentials. if registration user connection
 * succeeds, attempts auto-enrollment. if auto-enrollment is successful, stores
 * new user credentials in dataflash and attempts connection. if auto-enrollment
 * fails, board goes into error state.
 *
 * if m1user.txt provided user credentials, attempts connection
 * with those credentials. if connection fails, checks dataflash for
 * auto-enrolled credentials. if credentials are present in dataflash, attempts
 * connection with those credentials. if connection fails, attempts connection
 * with registration user credentials. if registration user credentials are not
 * available, board goes into error state. if registration user connection
 * succeeds, attempts auto-enrollment. if auto-enrollment is successful, stores
 * new user credentials in dataflash and attempts connection. if auto-enrollment
 * fails, board goes into error state.
 *
 * after successful connections, sends "on-connect" event and resumes sensor
 * threads. if diagnostics agent is enabled, configures sensor peripherals.
 * finally, enters infinite loop requesting sensor aggregates every sample_period.
 * if sensor aggregates are not received within 1 second of request, we assume
 * a sensor thread is dead-locked and reset the board. after receiving sensor
 * aggregates, publishes events for all sensor aggregates to the cloud.
 */
void net_thread_entry(void)
{
    ssp_err_t status = SSP_SUCCESS;
    char buf[400];
    char event[1536];
    const char * p_mqtt_broker_host = rna_broker_url;
    int ret = 0;
    int index;
    int i;
    uint16_t mqtt_broker_port = rna_port;
    ULONG start, actual_flags;
    fmi_product_info_t * effmi;
    app_config_t provisioned_config;
    user_credentials_t original_device;
#ifdef USE_TLS
    NX_SECURE_TLS_SESSION tls_session;
    NX_SECURE_X509_CERT ota_root_cert;
#endif
    ota_config_t ota_config = {
                               .processing_callback = toggle_green_led,
                               .net_config = {
#ifdef USE_TLS
                                              .p_tls_sesssion = &tls_session,
                                              .p_trusted_cert = &ota_root_cert,
#else
                                              .p_tls_session = NULL,
                                              .p_trusted_cert = NULL,
#endif
                                              .p_ip = &g_http_ip,
                                              .p_ppool = &g_http_packet_pool,
                                              .p_dns = &g_dns_client,
                               },
                               .file_config = {
                                               .p_media = &g_qspi_media,
                                               .filename = {0},
                                               .save_path = "Put binary here"
                               },
                               .crypto_config = {
                                                 .p_hash = &g_sce_hash_0,
                                                 .p_crypto = &g_sce_0
                               },

    };
#ifdef USE_M1DIAG
    const project_credentials_t diag_project = {
#ifdef DEV_DIAG
                                                .proj_id="kxwuKiOTycI",
                                                .apikey="2TXT7PJ4BGC4UA24XANNORBQGU2DIYTDG42DOMTDGQZDAMBQ"
#else
                                                .proj_id="Ohafs8Q31jU",
                                                .apikey="VRWM4JXNCKZEHIDP4T7TQRZQGU4TSOJQGI4WMZBQGQ3DAMBQ"
#endif
    };
    user_credentials_t original_diag_device;
    user_credentials_t diag_registration = {
#ifdef DEV_DIAG
                                            .user_id="1tyKNEkeZpQ",
#else
                                            .user_id="f_acx06uqDA",
#endif
                                            .password="Medium-1!"
    };

    original_diag_device.user_id[0] = '\0';
    original_diag_device.password[0] = '\0';
    app_config.diagnostics.user_id[0] = '\0';
    app_config.diagnostics.password[0] = '\0';
#endif

    original_device.user_id[0] = '\0';
    original_device.password[0] = '\0';
    memset(&provisioned_config, 0, sizeof(provisioned_config));
    g_fmi.p_api->productInfoGet(&effmi);
    status = g_flash0.p_api->open(g_flash0.p_ctrl, g_flash0.p_cfg);
    if (status)
        goto err;
    status = g_flash0.p_api->read(g_flash0.p_ctrl, (UCHAR *)buf, 0x40100000, sizeof(buf) - 1);
    if (status)
        goto err;
    status = g_flash0.p_api->close(g_flash0.p_ctrl);
    if (status)
        goto err;
    buf[sizeof(buf) - 1] = '\0';
    if (buf[0] == 'P') {
        // board may have been auto-provisioned; try provisioned credentials
        app_config_t * cfg_from_flash = (app_config_t *)&buf[1];
        memcpy(&provisioned_config, cfg_from_flash, sizeof(provisioned_config));
        if (strlen(app_config.project.apikey))
            memcpy(&cfg_from_flash->project, &app_config.project, sizeof(cfg_from_flash->project));
        if (strlen(app_config.device.password))
            memcpy(&cfg_from_flash->device, &app_config.device, sizeof(cfg_from_flash->device));
        if (strlen(app_config.registration.password))
            memcpy(&cfg_from_flash->registration, &app_config.registration, sizeof(cfg_from_flash->registration));
        memcpy(&app_config, cfg_from_flash, sizeof(app_config));
#ifdef USE_M1DIAG
        memcpy(&original_diag_device, &app_config.diagnostics, sizeof(original_diag_device));
#endif
        memcpy(&original_device, &app_config.device, sizeof(original_device));
    }

    set_leds(0, 1, 1);

    int idlen = 0;
    for (unsigned int k = 0; k < sizeof(effmi->unique_id); k++)
        idlen += sprintf(&event[idlen], "%02X", effmi->unique_id[k]);

#ifdef USE_M1DIAG
    m1_diag_initdata_t initdata;
    strcpy(initdata.email_address, "your@email.add");
    int sn_len = 0;
    for (unsigned int k = 0; k < sizeof(effmi->unique_id); k++)
        sn_len += sprintf(&initdata.serial_number[sn_len], "%02X", effmi->unique_id[k]);
    strcpy(initdata.model_number, "s5d9-revB");
    snprintf(initdata.fw_version, sizeof(initdata.fw_version), "%c%c%c%c%c%c%c%c-%d.%d",
            BUILD_YEAR_CH0, BUILD_YEAR_CH1, BUILD_YEAR_CH2, BUILD_YEAR_CH3,
            BUILD_MONTH_CH0, BUILD_MONTH_CH1,
            BUILD_DAY_CH0, BUILD_DAY_CH1,
            VERSION_MAJOR, VERSION_MINOR);
#ifdef DEV_DIAG
    ret = m1_diag_start("ladder.mediumone.com",
#ifdef USE_TLS
                            61614,
#else
                            61613,
#endif
#else
    ret = m1_diag_start("mqtt.mediumone.com",
#ifdef USE_TLS
                            61620,
#else
                            61619,
#endif
#endif
                            &diag_project,
                            &diag_registration,
                            &app_config.diagnostics,
                            event,
                            5,
                            5,
                            1000,
#ifdef USE_TLS
                            1,
                            diag_ssl_mem,
                            DIAG_SSL_MEM_SIZE,
#else
                            0,
                            NULL,
                            0,
#endif
                            &g_http_packet_pool,
                            &g_http_ip,
                            &g_dns_client,
                            900,
                            &initdata,
                            debug,
                            NULL);
    if ((ret == M1_SUCCESS) && strncmp(original_diag_device.password, app_config.diagnostics.password, sizeof(original_diag_device.password))) {
        ret = write_app_config_to_flash(&app_config);
        if (ret)
            goto err;
    }
#endif

    ret = m1_register_subscription_callback(m1_callback);
    ret = read_certs_and_connect(p_mqtt_broker_host, mqtt_broker_port, &app_config.project, &app_config.registration, &app_config.device);
    if (ret != M1_SUCCESS)
        goto err;

    set_leds(0, 0, 1);

    if (config_compare(&provisioned_config, &app_config)) {
        ret = write_app_config_to_flash(&app_config);
        if (ret)
            goto err;
    }

    unsigned long mac_address[2];
    nx_ip_interface_info_get(&g_http_ip, 0, NULL, NULL, NULL, NULL, &mac_address[0], &mac_address[1]);
    sprintf(event,
            "{\"firmware_version\":\"%c%c%c%c%c%c%c%c-%d.%d\",\"mac_address\":\"%02lX:%02lX:%02lX:%02lX:%02lX:%02lX\"}",
            BUILD_YEAR_CH0, BUILD_YEAR_CH1, BUILD_YEAR_CH2, BUILD_YEAR_CH3,
            BUILD_MONTH_CH0, BUILD_MONTH_CH1,
            BUILD_DAY_CH0, BUILD_DAY_CH1,
            VERSION_MAJOR, VERSION_MINOR,
            (mac_address[0] >> 8) & 0xff,
            mac_address[0] & 0xff,
            (mac_address[1] >> 24) & 0xff,
            (mac_address[1] >> 16) & 0xff,
            (mac_address[1] >> 8) & 0xff,
            mac_address[1] & 0xff);
    m1_publish_event(event, NULL);
    sprintf(event, "{\"link_code\":\"%s\"}", g_link_code);
    m1_publish_event(event, NULL);
#ifdef USE_M1DIAG
    M1_LOG(info, "M1 VSA connected", 0);
    sprintf((char *)buf, "{\"mqtt_user_id\":\"%s\",\"mqtt_project_id\":\"%s\",\"m1_api_key\":\"%s\"}",
            app_config.device.user_id,
            app_config.project.proj_id,
            app_config.project.apikey);
    m1_diag_update_configuration((char *)buf);
#endif

    status = g_i2c0.p_api->open(g_i2c0.p_ctrl, g_i2c0.p_cfg);
    if (status)
        goto err;

    tx_thread_resume(&adc_thread);
    tx_thread_resume(&bmc_thread);
    tx_thread_resume(&ens_thread);
    tx_thread_resume(&ms_thread);

    // added by Michael Li
    tx_thread_resume(&rfid_thread);

    g_timer1.p_api->open(g_timer1.p_ctrl, g_timer1.p_cfg);
    g_timer1.p_api->start(g_timer1.p_ctrl);

#ifdef USE_M1DIAG
    m1_diag_register_acceleration(&x_accel_access, &y_accel_access, &z_accel_access);
    m1_diag_register_humidity(&humidity_access);
    m1_diag_register_periphery("temp_ambient_c_1", M1_DIAG_FLOAT, &temp_access);
    m1_diag_register_periphery("x_magnetic_t", M1_DIAG_FLOAT, &xmag_access);
    m1_diag_register_periphery("y_magnetic_t", M1_DIAG_FLOAT, &ymag_access);
    m1_diag_register_periphery("z_magnetic_t", M1_DIAG_FLOAT, &zmag_access);
    m1_diag_register_periphery("temp_ambient_c_2", M1_DIAG_FLOAT, &temp2_access);
    m1_diag_register_periphery("temp_ambient_c_3", M1_DIAG_FLOAT, &temp3_access);
    m1_diag_register_periphery("pressure_mbar", M1_DIAG_FLOAT, &pressure_access);
    m1_diag_register_periphery("mic", M1_DIAG_FLOAT, &mic_access);
    m1_diag_register_periphery("voice_fft", M1_DIAG_FLOAT, &voice_fft_access);
#endif

    // Added by Michael Li (4/30/18 v1c)
    set_leds(0, 0, 1);  // turn on the green led
    sprintf(event, "{\"software_ver\":\"rfid_v1\"}");
    m1_publish_event(event, NULL);

    start = tx_time_get();

    while (1) {
        tx_thread_sleep(30);
        status = tx_event_flags_get(&g_sensor_event_flags, RFID_TRANSFER_REQUEST, TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER);
        if (status == TX_SUCCESS) {
            sprintf(event, "{\"card_key\":\"%s\"}", rfid_string);  // including null at the end.);
            m1_publish_event(event, NULL);
        }
    }

err:
#ifdef USE_M1DIAG
    M1_LOG(error, "Critical error (flash, i2c, or initial connection)", ret + (int)status);
#endif
    while (1) {
        set_leds(1, 1, 1);
        tx_thread_sleep(100);
        set_leds(0, 0, 0);
        tx_thread_sleep(100);
    }
}

void sensor_scheduler(timer_callback_args_t * p_args) {
    static unsigned int ticks = 0;
    ULONG flags_to_set = 0;

    if (p_args->event == TIMER_EVENT_EXPIRED) {
        ticks++;
        if (!(ticks % 3))  // 2 kHz (accelerometer) provided by sensor, but we only need 333.33 Hz
            flags_to_set |= BMC_SAMPLE_REQUEST;
        if (!(ticks % 131))  // 130 ms conversion time for relative humidity & temp
            flags_to_set |= ENS_SAMPLE_REQUEST;
        if (!(ticks % 35))  // 17 ms conversion-time at OSR = 8192, once for pressure and once for temperature
            flags_to_set |= MS_SAMPLE_REQUEST;
        if (flags_to_set)
            tx_event_flags_set(&g_sensor_event_flags, flags_to_set, TX_OR);
    }
}
