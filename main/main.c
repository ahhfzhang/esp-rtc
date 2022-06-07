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
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_log.h"

#include "board.h"
#include "console.h"
#include "audio_mem.h"
#include "periph_wifi.h"
#include "input_key_service.h"

#include "main_rtc_media.h"
#include "media_lib_adapter.h"
#include "esp_rtc.h"

#define TAG	        "ESP_RTC_Demo"

#define WIFI_SSID   "ESP-Audio"
#define WIFI_PWD    "esp123456"
#define LOGIN_URL   "tcp://100:100@192.168.1.123:5060"

static esp_rtc_handle_t esp_rtc;

static void setup_wifi(esp_periph_set_handle_t set)
{
    periph_wifi_cfg_t wifi_cfg = {
        .ssid = WIFI_SSID,
        .password = WIFI_PWD,
    };
    esp_periph_handle_t wifi_handle = periph_wifi_init(&wifi_cfg);
    esp_periph_start(set, wifi_handle);
    periph_wifi_wait_for_connected(wifi_handle, portMAX_DELAY);
}

static esp_err_t input_key_service_cb(periph_service_handle_t handle, periph_service_event_t *evt, void *ctx)
{
    audio_board_handle_t board_handle = (audio_board_handle_t) ctx;
    int player_volume;
    if (evt->type == INPUT_KEY_SERVICE_ACTION_CLICK_RELEASE) {
        ESP_LOGD(TAG, "[ * ] input key id is %d", (int)evt->data);
        switch ((int)evt->data) {
            case INPUT_KEY_USER_ID_REC:
                ESP_LOGI(TAG, "[ * ] [Rec] answer");
                esp_rtc_answer(esp_rtc);
                break;
            case INPUT_KEY_USER_ID_MUTE:
                ESP_LOGI(TAG, "[ * ] [Mute] bye");
                esp_rtc_bye(esp_rtc);
                break;
            case INPUT_KEY_USER_ID_PLAY:
                ESP_LOGI(TAG, "[ * ] [Play] calling");
                esp_rtc_call(esp_rtc, "1010");
                break;
            case INPUT_KEY_USER_ID_MODE:
            case INPUT_KEY_USER_ID_SET:
                ESP_LOGI(TAG, "[ * ] [Set] input key event");
                esp_rtc_bye(esp_rtc);
                break;
            case INPUT_KEY_USER_ID_VOLUP:
                ESP_LOGD(TAG, "[ * ] [Vol+] input key event");
                audio_hal_get_volume(board_handle->audio_hal, &player_volume);
                player_volume += 10;
                if (player_volume > 100) {
                    player_volume = 100;
                }
                audio_hal_set_volume(board_handle->audio_hal, player_volume);
                ESP_LOGI(TAG, "[ * ] Volume set to %d %%", player_volume);
                break;
            case INPUT_KEY_USER_ID_VOLDOWN:
                ESP_LOGD(TAG, "[ * ] [Vol-] input key event");
                audio_hal_get_volume(board_handle->audio_hal, &player_volume);
                player_volume -= 10;
                if (player_volume < 0) {
                    player_volume = 0;
                }
                audio_hal_set_volume(board_handle->audio_hal, player_volume);
                ESP_LOGI(TAG, "[ * ] Volume set to %d %%", player_volume);
                break;
        }
    }

    return ESP_OK;
}

static struct {
    struct arg_str *phone_num;
    struct arg_end *end;
} invite_args;

static int sip_invite(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &invite_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, invite_args.end, argv[0]);
        return 1;
    }

    esp_rtc_call(esp_rtc, invite_args.phone_num->sval[0]);
    return 0;
}

static int sip_answer(int argc, char **argv)
{
    esp_rtc_answer(esp_rtc);
    return 0;
}

static int sip_bye(int argc, char **argv)
{
    esp_rtc_bye(esp_rtc);
    return 0;
}

void app_main()
{
    esp_log_level_set("*", ESP_LOG_INFO);
    AUDIO_MEM_SHOW(TAG);

	/* tcp/ip init */
	esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_netif_init());

    media_lib_add_default_adapter();

    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    periph_cfg.task_stack = 3 * 1024;
    periph_cfg.extern_stack = true;
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);

    audio_board_handle_t board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);
    audio_hal_set_volume(board_handle->audio_hal, 65);

    setup_wifi(set);
    setup_audio();
    setup_lcd(set);

    AUDIO_MEM_SHOW(TAG);
    esp_rtc = main_rtc_start(LOGIN_URL);

    audio_board_key_init(set);
    input_key_service_info_t input_key_info[] = INPUT_KEY_DEFAULT_INFO();
    input_key_service_cfg_t input_cfg = INPUT_KEY_SERVICE_DEFAULT_CONFIG();
    input_cfg.handle = set;
    input_cfg.based_cfg.task_stack = 4 * 1024;
    input_cfg.based_cfg.extern_stack = true;
    periph_service_handle_t input_ser = input_key_service_create(&input_cfg);
    input_key_service_add_key(input_ser, input_key_info, INPUT_KEY_NUM);
    periph_service_set_callback(input_ser, input_key_service_cb, (void *)board_handle);

    console_init();
    invite_args.phone_num   = arg_str1(NULL, NULL, "<phone_num>", "invite phone number");
    invite_args.end         = arg_end(2);
    const esp_console_cmd_t cmd_invite = {
        .command = "call",
        .help = "Make a Call",
        .hint = NULL,
        .func = &sip_invite,
        .argtable = &invite_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_invite) );

    const esp_console_cmd_t cmd_sip_bye = {
        .command = "bye",
        .help = "BYE",
        .hint = NULL,
        .func = &sip_bye,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_sip_bye));

    const esp_console_cmd_t cmd_sip_answer = {
        .command = "answer",
        .help = "Answer the call",
        .hint = NULL,
        .func = &sip_answer,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_sip_answer));
}
