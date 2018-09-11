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
#include <stdio.h>

#define ARM_MATH_CM4
#include "arm_math.h"
#include "adc_thread.h"
#include "net_thread.h"
#include "nxd_dns.h"
#include "fx_stdio.h"
#include <i2c.h>
#include "led.h"
#include "app.h"
#include "agg.h"
#include "m1_agent.h"
#ifdef USE_M1DIAG
#include "m1diagnostics_agent.h"
#endif


#define ADC_BASE_PTR  R_S12ADC0_Type *
#define AVG_CNT 2000        // Sampling at 4Khz, average over .5 sec

// Supported FFT Lengths are 32, 64, 128, 256, 512, 1024, 2048, 4096
// for now we are fixing the sample rate at 4 kHz, though it can be increased significantly
#define ADC_FS 4000
/* The work in this paper http://www.utdallas.edu/ssprl/files/ConferencePaperVADapp.pdf
 * uses frame lengths around 12.5 ms, not sure why. 12.5 ms @ 4000 kHz sampling rate
 * is 50 samples. we have to take 2x the frame length for fft processing, and the closest
 * supported fft size would then be 128, so we will set our "input" frame length to be 64 samples
*/
#define FFT_SIZE 4096
#define INPUT_FRAME_LENGTH_US 12500
#define INPUT_FRAME_LENGTH_SAMPLES (ADC_FS * FRAME_LENGTH_US) / 1000000  // 50 samples @ 4 kHz and 12.5 ms frame length
#define PROCESSING_FRAME_LENGTH_SAMPLES 2 * INPUT_FRAME_LENGTH_SAMPLES

#ifdef USE_M1DIAG
volatile agg_t mic = { .name = "mic", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t voice_fft = { .name = "voice_fft", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
#endif

static volatile uint16_t sound_level = 0;
static float32_t sample_buf[FFT_SIZE];
static float32_t Output[FFT_SIZE];
static ULONG fft_cnt = 0L;

// TODO: these 2 agg arrays dont need to be FFT_SIZE; only FFT_WIDTH
static float64_t fft_avg[FFT_SIZE] = {0};
static float32_t fft_max[FFT_SIZE] = {0};

static volatile uint8_t sample = 1;
#define FFT_PRECISION 1
#define FFT_WIDTH 175
#define FFT_START (280 - FFT_WIDTH)
static char fft_msg[10 + (10 + FFT_PRECISION) * FFT_WIDTH - 1 + 3];


/******************************************************************************
* Function Name: adc_thread_entry
* Description  : Thread begins execution after being resumed by net_thread,
*                after successful connection to the cloud. Configures the
*                ADC (connected to mic). After 150000 ADC samples, takes the
*                peak-peak (max - min) and adds to the running aggregate.
*                Aggregate is sent to the cloud when ADC_TRANSFER_REQUEST is
*                received. Aggregate contains:
*                    - min
*                    - max
*                    - average
*                    - last sample value
******************************************************************************/
void adc_thread_entry(void)
{
    ssp_err_t err;
    ULONG actual_flags;
    ADC_BASE_PTR p_adc;
    adc_instance_ctrl_t * p_ctrl = (adc_instance_ctrl_t *) g_adc0.p_ctrl;
#ifndef USE_M1DIAG
    agg_t mic = { .name = "mic", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
    agg_t voice_fft = { .name = "voice_fft", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
#endif

    p_adc = (ADC_BASE_PTR) p_ctrl->p_reg;
    /** Disable differential inputs */
    p_adc->ADPGADCR0 = 0;
    /** Bypass PGA amplifier */
    p_adc->ADPGACR = 0x0111;

    err = g_sf_adc_periodic0.p_api->start(g_sf_adc_periodic0.p_ctrl);
    if (err != SSP_SUCCESS) {
#ifdef USE_M1DIAG
        M1_LOG(error, "Unable to start ADC periodic framework", err);
#endif
        return;
    }

    // TODO: do we need to re-init for every transform?
    arm_rfft_fast_instance_f32 S;
    arm_status status;
    status = arm_rfft_fast_init_f32(&S, FFT_SIZE);

    while (1) {
        err = tx_event_flags_get(&g_sensor_event_flags, ADC_TRANSFER_REQUEST | ADC_THRESHOLD_UPDATE | ADC_BATCH_COMPLETE | ADC_FFT_BATCH_COMPLETE, TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER);
        if (err == TX_SUCCESS) {
            if (actual_flags & ADC_BATCH_COMPLETE)
                update_agg(&mic, sound_level);
            if (actual_flags & ADC_FFT_BATCH_COMPLETE) {
                uint8_t rfftMode = 0;

                if (status == 0) {
                    arm_rfft_fast_f32(&S, sample_buf, Output, rfftMode);
                    sample = 1;
                    arm_cmplx_mag_f32(Output, Output, FFT_SIZE / 2);
                    for (int i = 0; i < FFT_WIDTH; i++) {
                        fft_avg[i + FFT_START] += Output[i + FFT_START];
                        if (fft_max[i + FFT_START] < Output[i + FFT_START])
                            fft_max[i + FFT_START] = Output[i + FFT_START];
                    }
                    fft_cnt++;
                }
            } if (actual_flags & ADC_THRESHOLD_UPDATE) {
                update_threshold((agg_t *)&g_mic, &mic);
            }
            if (actual_flags & ADC_TRANSFER_REQUEST) {
                transfer_agg(&mic, &g_mic);
                tx_event_flags_set(&g_sensor_event_flags, ADC_TRANSFER_COMPLETE, TX_OR);
                reset_agg(&mic);

                // bin size: 4 kHz sample rate / 4096 samples = 0.976 Hz / bin
                // male low: 85 Hz
                // female high: ? Hz
                int fft_msg_index = sprintf(fft_msg, "{\"mic_fft_max\":[");
                for (int i = 0; i < FFT_WIDTH; i++) {
                    fft_msg_index += sprintf(fft_msg + fft_msg_index, "%.1e,", fft_max[i + FFT_START]);
                    fft_max[i + FFT_START] = 0;
                }
                fft_msg_index += sprintf(fft_msg + fft_msg_index - 1, "]}");
                m1_publish_event(fft_msg, NULL);

                if (fft_cnt) {
                    fft_msg_index = sprintf(fft_msg, "{\"mic_fft_avg\":[");
                    for (int i = 0; i < FFT_WIDTH; i++) {
                        fft_msg_index += sprintf(fft_msg + fft_msg_index, "%.1e,", fft_avg[i + FFT_START] / fft_cnt);
                        fft_avg[i + FFT_START] = 0;
                    }
                    fft_cnt = 0;
                    fft_msg_index += sprintf(fft_msg + fft_msg_index - 1, "]}");
                    m1_publish_event(fft_msg, NULL);
                }
            }
        }
    }
}

void g_adc_framework_user_callback(sf_adc_periodic_callback_args_t * p_args) {
    static uint16_t sample_count = 0, fft_sample_count = 0;
    static uint16_t max = 0U;
    static uint16_t min = 4095U;
    uint16_t data = g_adc_raw_data[p_args->buffer_index];

    if (data > max)
        max = data;
    if (data < min)
        min = data;


    if (sample_count++ > AVG_CNT) {
        sample_count = 0;
        sound_level = (uint16_t) (max-min);
        max = 0U;
        min = 4095U;
        tx_event_flags_set(&g_sensor_event_flags, ADC_BATCH_COMPLETE, TX_OR);
    }

    if (!sample)
        return;

    sample_buf[fft_sample_count++] = data;

    if (fft_sample_count >= FFT_SIZE) {
        fft_sample_count = 0;
        sample = 0;
        tx_event_flags_set(&g_sensor_event_flags, ADC_FFT_BATCH_COMPLETE, TX_OR);
    }

}
