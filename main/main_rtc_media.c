/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2022 <ESPRESSIF SYSTEMS (SHANGHAI) CO., LTD>
 *
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include "string.h"
#include "board.h"
#include "esp_log.h"
#include "audio_pipeline.h"
#include "audio_element.h"
#include "audio_thread.h"
#include "audio_mem.h"
#include "filter_resample.h"
#include "i2s_stream.h"
#include "raw_stream.h"
#include "algorithm_stream.h"
#include "fatfs_stream.h"
#include "wav_encoder.h"
#include "g711_decoder.h"
#include "g711_encoder.h"
#include "i2c_bus.h"
#include "es7210.h"

#include "esp_lcd_panel_ops.h"
#include "esp_camera.h"
#include "esp_jpeg_dec.h"
#include "esp_jpeg_enc.h"
#include "esp_rtc.h"

#define TAG	        "ESP_RTC_MEDIA"

#define SOFTWARE_ENC
#define VIDEO_FPS                   15
#define VIDEO_FRAME_SIZE            FRAMESIZE_QVGA
#define VIDEO_SIZE                  50*1024

#define I2S_SAMPLE_RATE             8000
#define I2S_CHANNELS                1
#define I2S_BITS                    16

#define CODEC_SAMPLE_RATE           8000
#define CODEC_CHANNELS              1

/* Debug original input data for AEC feature*/
// #define DEBUG_AEC_INPUT

#define ESP_READ_BUFFER_SIZE    1024

extern const uint8_t espressif_jpg_start[] asm("_binary_espressif_jpg_start");
extern const uint8_t espressif_jpg_end[]   asm("_binary_espressif_jpg_end");
extern const uint8_t keyboard_jpg_start[] asm("_binary_keyboard_jpg_start");
extern const uint8_t keyboard_jpg_end[]   asm("_binary_keyboard_jpg_end");
extern const uint8_t calling_jpg_start[] asm("_binary_calling_jpg_start");
extern const uint8_t calling_jpg_end[]   asm("_binary_calling_jpg_end");
extern const uint8_t incoming_jpg_start[] asm("_binary_incoming_jpg_start");
extern const uint8_t incoming_jpg_end[]   asm("_binary_incoming_jpg_end");

const int ENCODER_STOPPED_BIT = BIT0;

typedef struct {
    unsigned char *buf;
    int size;
} jpeg_enc_msg_t;

typedef struct {
    bool enc_run;
    void *jpeg_enc;
    uint32_t imageCnt;
    uint32_t sysTime;
    unsigned char *decBuf;
    QueueHandle_t encQ;
    EventGroupHandle_t state;
    jpeg_dec_io_t *jpeg_io;
    jpeg_dec_handle_t jpeg_dec;
    jpeg_dec_header_info_t *out_info;
    audio_element_handle_t raw_read;
    audio_element_handle_t raw_write;
    audio_pipeline_handle_t recorder;
    audio_pipeline_handle_t player;
    esp_rtc_handle_t esp_rtc;
} main_rtc_t;

static main_rtc_t *rtc;
static esp_lcd_panel_handle_t panel_handle;

static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,
#ifdef SOFTWARE_ENC
    .xclk_freq_hz = 24000000,
#else
    .xclk_freq_hz = 20000000,
#endif
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

#ifdef SOFTWARE_ENC
    .pixel_format = PIXFORMAT_YUV422,
#else
    .pixel_format = PIXFORMAT_JPEG,
#endif
    .frame_size = VIDEO_FRAME_SIZE,

    .jpeg_quality = 12,     //0-63 lower number means higher quality
    .fb_count = 2,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

static esp_err_t es7210_write_reg(i2c_bus_handle_t i2c_handle, uint8_t reg_addr, uint8_t data)
{
    return i2c_bus_write_bytes(i2c_handle, ES7210_AD1_AD0_00, &reg_addr, sizeof(reg_addr), &data, sizeof(data));
}

int setup_audio()
{
    es7210_mic_select(ES7210_INPUT_MIC1 | ES7210_INPUT_MIC2 | ES7210_INPUT_MIC3 | ES7210_INPUT_MIC4);
    i2c_config_t es_i2c_cfg;
    i2c_bus_handle_t i2c_handle = i2c_bus_create(I2C_NUM_0, &es_i2c_cfg);
    es7210_write_reg(i2c_handle, 0x01, 0x20);
    es7210_write_reg(i2c_handle, 0x03, 0x06);
    es7210_write_reg(i2c_handle, 0x04, 0x03);
    es7210_write_reg(i2c_handle, 0x06, 0x04);
    es7210_write_reg(i2c_handle, 0x08, 0x14);
    es7210_write_reg(i2c_handle, 0x0b, 0x01);
    es7210_write_reg(i2c_handle, 0x11, 0x60);
    es7210_write_reg(i2c_handle, 0x12, 0x02);
    es7210_write_reg(i2c_handle, 0x3f, 0x01);
    es7210_write_reg(i2c_handle, 0x43, 0x1e);
    es7210_write_reg(i2c_handle, 0x44, 0x1e);
    es7210_write_reg(i2c_handle, 0x45, 0x18);
    es7210_write_reg(i2c_handle, 0x46, 0x1e);
    es7210_write_reg(i2c_handle, 0x47, 0x08);
    es7210_write_reg(i2c_handle, 0x49, 0x08);
    es7210_write_reg(i2c_handle, 0x4a, 0x08);

    return ESP_OK;
}

static void init_jpeg_encoder()
{
    jpeg_enc_info_t info = { 0 };
    info.width = resolution[VIDEO_FRAME_SIZE].width;
    info.height = resolution[VIDEO_FRAME_SIZE].height;
    info.src_type = JPEG_RAW_TYPE_YCbYCr;
    info.subsampling = JPEG_SUB_SAMPLE_YUV420;
    info.quality = 40;
    // info.task_enable = true;
    info.hfm_task_core = 1;
    info.hfm_task_priority = 15;
    rtc->jpeg_enc = jpeg_enc_open(&info);
}

static int decode_and_render(const unsigned char *src, uint32_t size, bool need_free)
{
    int ret = 0;
    int outbuf_len = 320 * 240 * 2;

    if (rtc->jpeg_dec == NULL) {
        // Generate default configuration
        jpeg_dec_config_t config = DEFAULT_JPEG_DEC_CONFIG();
        // Create jpeg_dec
        rtc->jpeg_dec = jpeg_dec_open(&config);
        // Create io_callback handle
        rtc->jpeg_io = audio_calloc(1, sizeof(jpeg_dec_io_t));
        if (rtc->jpeg_io == NULL) {
            return ESP_FAIL;
        }
        // Create out_info handle
        rtc->out_info = audio_calloc(1, sizeof(jpeg_dec_header_info_t));
        if (rtc->out_info == NULL) {
            return ESP_FAIL;
        }
        rtc->decBuf = audio_calloc(1, outbuf_len);
        if (rtc->decBuf == NULL) {
            return ESP_FAIL;
        }
    }

    // Set input buffer and buffer len to io_callback
    rtc->jpeg_io->inbuf = (unsigned char *)src;
    rtc->jpeg_io->inbuf_len = size;
    // Parse jpeg picture header and get picture for user and decoder
    ret = jpeg_dec_parse_header(rtc->jpeg_dec, rtc->jpeg_io, rtc->out_info);
    if (ret < 0) {
        // ESP_LOGE(TAG, "Got an error by jpeg_dec_parse_header, ret:%d", ret);
        return ret;
    }

    ESP_LOGD(TAG, "The image size is %d bytes, width:%d, height:%d", outbuf_len, rtc->out_info->width, rtc->out_info->height);

    rtc->jpeg_io->outbuf = rtc->decBuf;
    int inbuf_consumed = rtc->jpeg_io->inbuf_len - rtc->jpeg_io->inbuf_remain;
    rtc->jpeg_io->inbuf = (unsigned char *)(src + inbuf_consumed);
    rtc->jpeg_io->inbuf_len = rtc->jpeg_io->inbuf_remain;

    // Start decode jpeg raw data
    ret = jpeg_dec_process(rtc->jpeg_dec, rtc->jpeg_io);
    if (ret < 0) {
        ESP_LOGE(TAG, "Got an error by jpeg_dec_process, ret:%d", ret);
        return ret;
    }

    uint16_t *p = (uint16_t *)rtc->decBuf;
    for (int i = 0; i < 320 * 240; ++i) {
        *p = (*p >> 8) | (*p << 8);
        p++;
    }

    for (int y = 0; y < 240; y += 40) {
        esp_lcd_panel_draw_bitmap(panel_handle, 0, y, 320, y + 40, rtc->decBuf + y * (320*2));
    }

    // Decoder deinitialize
    if (need_free && rtc->jpeg_dec) {
        jpeg_dec_close(rtc->jpeg_dec);
        audio_free(rtc->out_info);
        audio_free(rtc->jpeg_io);
        audio_free(rtc->decBuf);
        rtc->jpeg_dec = NULL;
    }
    return ESP_OK;
}

static void show_jpg(int type)
{
    switch (type)
    {
        case 1:
            decode_and_render(espressif_jpg_start, espressif_jpg_end - espressif_jpg_start, true);
            break;
        case 2:
            decode_and_render(keyboard_jpg_start, keyboard_jpg_end - keyboard_jpg_start, true);
            break;
        case 3:
            decode_and_render(calling_jpg_start, calling_jpg_end - calling_jpg_start, true);
            break;
        case 4:
            decode_and_render(incoming_jpg_start, incoming_jpg_end - incoming_jpg_start, true);
            break;
    }
}

int setup_lcd(esp_periph_set_handle_t set)
{
    panel_handle = audio_board_lcd_init(set, NULL);

#ifdef DEBUG_AEC_INPUT
    audio_board_sdcard_init(set, SD_MODE_1_LINE);
#endif

    return ESP_OK;
}

// Get the playback reference data and mic data from the four-channel data
static esp_err_t i2s_data_divided(int16_t *raw_buff, int len, int16_t *buf_aec)
{
    for (int i = 0; i < len / 4; i++) {
        buf_aec[i << 1]         = raw_buff[(i << 1) + 1];
        buf_aec[(i << 1) + 1]   = raw_buff[i << 1];
    }
    return ESP_OK;
}

int i2s_stream_read_cb(audio_element_handle_t el, char *buf, int len, TickType_t wait_time, void *ctx)
{
    size_t bytes_read = 0;

    char *buf_tmp = audio_calloc(1, ESP_READ_BUFFER_SIZE);
    AUDIO_MEM_CHECK(TAG, buf, return 0 );
    char *buf_aec = audio_calloc(1, ESP_READ_BUFFER_SIZE);
    AUDIO_MEM_CHECK(TAG, buf_aec, return 0);

    i2s_read(0, buf_tmp, ESP_READ_BUFFER_SIZE, &bytes_read, wait_time);
    if (bytes_read == ESP_READ_BUFFER_SIZE) {
        i2s_data_divided((int16_t *)buf_tmp, ESP_READ_BUFFER_SIZE, (int16_t *)buf_aec);
        memcpy(buf, buf_aec, bytes_read);
    }

    free(buf_tmp);
    free(buf_aec);
    return bytes_read;
}

static esp_err_t recorder_pipeline_open()
{
    audio_element_handle_t i2s_stream_reader;
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    rtc->recorder = audio_pipeline_init(&pipeline_cfg);
    AUDIO_NULL_CHECK(TAG, rtc->recorder, return ESP_FAIL);

    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_READER;
    i2s_cfg.uninstall_drv = false;
    i2s_cfg.task_core = 1;
    i2s_cfg.i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    i2s_cfg.i2s_config.sample_rate = I2S_SAMPLE_RATE;
    i2s_cfg.i2s_config.bits_per_sample = 32;
    i2s_cfg.out_rb_size = 2 * 1024;
    i2s_cfg.stack_in_ext = true;
    i2s_stream_reader = i2s_stream_init(&i2s_cfg);
    audio_element_set_read_cb(i2s_stream_reader, i2s_stream_read_cb, NULL);

    algorithm_stream_cfg_t algo_config = ALGORITHM_STREAM_CFG_DEFAULT();
    algo_config.input_type = ALGORITHM_STREAM_INPUT_TYPE1;
    algo_config.task_core = 1;
#ifdef DEBUG_AEC_INPUT
    algo_config.debug_input = true;
#endif
    audio_element_handle_t element_algo = algo_stream_init(&algo_config);
    audio_element_set_music_info(element_algo, I2S_SAMPLE_RATE, 1, I2S_BITS);

    audio_pipeline_register(rtc->recorder, i2s_stream_reader, "i2s");
    audio_pipeline_register(rtc->recorder, element_algo, "algo");

#ifdef DEBUG_AEC_INPUT
    wav_encoder_cfg_t wav_cfg = DEFAULT_WAV_ENCODER_CONFIG();
    wav_cfg.task_core = 1;
    audio_element_handle_t wav_encoder = wav_encoder_init(&wav_cfg);

    fatfs_stream_cfg_t fatfs_wd_cfg = FATFS_STREAM_CFG_DEFAULT();
    fatfs_wd_cfg.type = AUDIO_STREAM_WRITER;
    fatfs_wd_cfg.task_core = 1;
    audio_element_handle_t fatfs_stream_writer = fatfs_stream_init(&fatfs_wd_cfg);

    audio_pipeline_register(rtc->recorder, wav_encoder, "wav_enc");
    audio_pipeline_register(rtc->recorder, fatfs_stream_writer, "fatfs_stream");

    const char *link_tag[4] = {"i2s", "algo", "wav_enc", "fatfs_stream"};
    audio_pipeline_link(rtc->recorder, &link_tag[0], 4);

    audio_element_info_t fat_info = {0};
    audio_element_getinfo(fatfs_stream_writer, &fat_info);
    fat_info.sample_rates = I2S_SAMPLE_RATE;
    fat_info.bits = ALGORITHM_STREAM_DEFAULT_SAMPLE_BIT;
    fat_info.channels = 2;
    audio_element_setinfo(fatfs_stream_writer, &fat_info);
    audio_element_set_uri(fatfs_stream_writer, "/sdcard/aec_in.wav");
#else

    g711_encoder_cfg_t g711_cfg = DEFAULT_G711_ENCODER_CONFIG();
    g711_cfg.task_core = 1;
    audio_element_handle_t g711_encoder = g711_encoder_init(&g711_cfg);

    raw_stream_cfg_t raw_cfg = RAW_STREAM_CFG_DEFAULT();
    raw_cfg.type = AUDIO_STREAM_READER;
    raw_cfg.out_rb_size = 2 * 1024;
    rtc->raw_read = raw_stream_init(&raw_cfg);
    audio_element_set_output_timeout(rtc->raw_read, portMAX_DELAY);

    audio_pipeline_register(rtc->recorder, g711_encoder, "g711Enc");
    audio_pipeline_register(rtc->recorder, rtc->raw_read, "raw");

    const char *link_tag[4] = {"i2s", "algo", "g711Enc", "raw"};
    audio_pipeline_link(rtc->recorder, &link_tag[0], 4);
#endif

    ESP_LOGI(TAG, "RTC recorder has been created");
    return ESP_OK;
}

static esp_err_t player_pipeline_open()
{
    audio_element_handle_t i2s_stream_writer;
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    rtc->player = audio_pipeline_init(&pipeline_cfg);
    AUDIO_NULL_CHECK(TAG, rtc->player, return ESP_FAIL);

    raw_stream_cfg_t raw_cfg = RAW_STREAM_CFG_DEFAULT();
    raw_cfg.type = AUDIO_STREAM_WRITER;
    raw_cfg.out_rb_size = 1024;
    rtc->raw_write = raw_stream_init(&raw_cfg);

    g711_decoder_cfg_t g711_cfg = DEFAULT_G711_DECODER_CONFIG();
    g711_cfg.task_core = 1;
    audio_element_handle_t g711_decoder = g711_decoder_init(&g711_cfg);

    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_WRITER;
    i2s_cfg.uninstall_drv = false;
    i2s_cfg.i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    i2s_cfg.i2s_config.sample_rate = I2S_SAMPLE_RATE;
    i2s_cfg.i2s_config.bits_per_sample = 32;
    i2s_cfg.need_expand = true;
    i2s_cfg.stack_in_ext = true;
    i2s_cfg.task_core = 1;
    i2s_stream_writer = i2s_stream_init(&i2s_cfg);

    audio_pipeline_register(rtc->player, rtc->raw_write, "raw");
    audio_pipeline_register(rtc->player, g711_decoder, "g711Dec");
    audio_pipeline_register(rtc->player, i2s_stream_writer, "i2s");
    const char *link_tag[3] = {"raw", "g711Dec", "i2s"};
    audio_pipeline_link(rtc->player, &link_tag[0], 3);

    ESP_LOGI(TAG, "RTC player has been created");
    return ESP_OK;
}

static uint32_t _sysnow()
{
    return (xTaskGetTickCount() * portTICK_PERIOD_MS) / 1000;
}

static void _videoEnc(void* pv)
{
    xEventGroupClearBits(rtc->state, ENCODER_STOPPED_BIT);
    while (rtc->enc_run) {
        camera_fb_t *pic = esp_camera_fb_get();
        if (pic) {
            jpeg_enc_msg_t enc;
            enc.buf = audio_calloc(1, VIDEO_SIZE);
            #ifdef SOFTWARE_ENC
            jpeg_enc_process(rtc->jpeg_enc, pic->buf, pic->len, enc.buf, VIDEO_SIZE, &enc.size);
            #else
            memcpy(enc.buf, pic->buf, pic->len);
            enc.size = pic->len;
            #endif
            esp_camera_fb_return(pic);
            if (xQueueSend(rtc->encQ, &enc, 100 / portTICK_PERIOD_MS) != pdTRUE) {
                ESP_LOGW(TAG, "send enc buf queue timeout !");
                free(enc.buf);
            }
        } else {
            vTaskDelay(5 / portTICK_PERIOD_MS);
        }
    }
    ESP_LOGI(TAG, "_videoEnc task stoped");
    xEventGroupSetBits(rtc->state, ENCODER_STOPPED_BIT);
    vTaskDelete(NULL);
}

static esp_err_t video_start()
{
    esp_camera_init(&camera_config);
    init_jpeg_encoder();

    rtc->enc_run = true;
    rtc->encQ = xQueueCreate(1, sizeof(jpeg_enc_msg_t));
    if (audio_thread_create(NULL, "_videoEnc", _videoEnc, NULL, 3*1024, 20, true, 0) != ESP_OK) {
        ESP_LOGE(TAG, "Can not start _videoEnc task");
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t video_stop()
{
    rtc->enc_run = false;
    xEventGroupWaitBits(rtc->state, ENCODER_STOPPED_BIT, false, true, portMAX_DELAY);

    jpeg_enc_close(rtc->jpeg_enc);
    esp_camera_deinit();

    jpeg_enc_msg_t enc;
    if (xQueueReceive(rtc->encQ, &enc, 0) == pdTRUE) {
        audio_free(enc.buf);
    }
    vQueueDelete(rtc->encQ);
    return ESP_OK;
}

static int _send_audio(unsigned char *data, int len)
{
#ifdef DEBUG_AEC_INPUT
    vTaskDelay(20 / portTICK_PERIOD_MS);
    return len;
#endif
    return raw_stream_read(rtc->raw_read, (char *)data, len);
}

static int _receive_audio(unsigned char *data, int len)
{
    return raw_stream_write(rtc->raw_write, (char *)data, len);
}

static int _send_video(unsigned char *data, unsigned int *len)
{
    jpeg_enc_msg_t enc;
    if (xQueueReceive(rtc->encQ, &enc, 0) != pdTRUE) {
        *len = 0;
        // ESP_LOGW(TAG, "Can not get enc buffer timely!");
    } else {
        *len = enc.size;
        memcpy(data, enc.buf, enc.size);
        audio_free(enc.buf);
        rtc->imageCnt ++;
        if ((rtc->imageCnt%250) == 0) {
            if (rtc->sysTime) {
                ESP_LOGI(TAG, "send video %.2f fps !", (float)rtc->imageCnt/(_sysnow() - rtc->sysTime));
            }
            rtc->sysTime = _sysnow();
            rtc->imageCnt = 0;
        }
        // ESP_LOGI(TAG, "send video %d !", enc.size);
    }

    return ESP_OK;
}

static int _receive_video(unsigned char *data, int len)
{
    return decode_and_render(data, len, false);
}

static int _esp_rtc_event_handler(esp_rtc_event_t event)
{
    ESP_LOGD(TAG, "_esp_rtp_event_handler event %d", event);
    switch ((int)event) {
        case ESP_RTC_EVENT_REGISTERED:
            ESP_LOGI(TAG, "ESP_RTC_EVENT_REGISTERED");
            show_jpg(2);
            break;
        case ESP_RTC_EVENT_UNREGISTERED:
            ESP_LOGI(TAG, "ESP_RTC_EVENT_UNREGISTERED");
            show_jpg(1);
            break;
        case ESP_RTC_EVENT_CALLING:
            ESP_LOGI(TAG, "ESP_RTC_EVENT_CALLING...");
            show_jpg(3);
            break;
        case ESP_RTC_EVENT_INCOMING:
            ESP_LOGI(TAG, "ESP_RTC_EVENT_INCOMING...");
            show_jpg(4);
            break;
        case ESP_RTC_EVENT_AUDIO_SESSION_BEGIN:
            ESP_LOGI(TAG, "ESP_RTC_EVENT_AUDIO_SESSION_BEGIN");
            recorder_pipeline_open();
            player_pipeline_open();
            audio_pipeline_run(rtc->player);
            audio_pipeline_run(rtc->recorder);
            break;
        case ESP_RTC_EVENT_AUDIO_SESSION_END:
            ESP_LOGI(TAG, "ESP_RTC_EVENT_AUDIO_SESSION_END");
            audio_pipeline_stop(rtc->recorder);
            audio_pipeline_wait_for_stop(rtc->recorder);
            audio_pipeline_deinit(rtc->recorder);
            audio_pipeline_stop(rtc->player);
            audio_pipeline_wait_for_stop(rtc->player);
            audio_pipeline_deinit(rtc->player);
            break;
        case ESP_RTC_EVENT_VIDEO_SESSION_BEGIN:
            ESP_LOGI(TAG, "ESP_RTC_EVENT_VIDEO_SESSION_BEGIN");
            video_start();
            break;
        case ESP_RTC_EVENT_VIDEO_SESSION_END:
            ESP_LOGI(TAG, "ESP_RTC_EVENT_VIDEO_SESSION_END");
            video_stop();
            break;
        case ESP_RTC_EVENT_HANGUP:
            ESP_LOGI(TAG, "ESP_RTC_EVENT_HANGUP");
            show_jpg(2);
            break;
        case ESP_RTC_EVENT_ERROR:
            ESP_LOGI(TAG, "ESP_RTC_EVENT_ERROR");
            break;
    }

    return 0;
}

esp_rtc_handle_t main_rtc_start(const char *uri)
{
    AUDIO_NULL_CHECK(TAG, uri, return NULL);
    rtc = audio_calloc(1, sizeof(main_rtc_t));
    AUDIO_NULL_CHECK(TAG, rtc, return NULL);

    vTaskDelay(100 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Booting logo ......");
    show_jpg(1);

    esp_rtc_video_info vCodecInfo = {
        .vcodec = RTC_VCODEC_JPEG,
        .width = resolution[VIDEO_FRAME_SIZE].width,
        .height = resolution[VIDEO_FRAME_SIZE].height,
        .fps = VIDEO_FPS,
        .len = VIDEO_SIZE,
    };
    esp_rtc_data_cb_t data_cb = {
        .send_audio = _send_audio,
        .receive_audio = _receive_audio,
        .send_video = _send_video,
        .receive_video = _receive_video,
    };
    esp_rtc_config_t rtc_media_config = {
        .uri = uri,
        .aCodecType = RTC_ACODEC_G711A,
        .vCodecInfo = &vCodecInfo,
        .dataCb = &data_cb,
        .usePublicAddr = false,
        .eventHandler = _esp_rtc_event_handler,
    };

    rtc->state = xEventGroupCreate();
    rtc->esp_rtc = esp_rtc_init(&rtc_media_config);

    return rtc->esp_rtc;
}
