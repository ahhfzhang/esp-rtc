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

#ifndef _ESP_RTC_H_
#define _ESP_RTC_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _esp_rtc_handle *esp_rtc_handle_t;

/**
 * @brief RTC codec type
 */
typedef enum {
    RTC_ACODEC_G711A,
    RTC_ACODEC_G711U,
    RTC_VCODEC_JPEG,
} rtc_payload_codec_t;

typedef enum {
    ESP_RTC_EVENT_NULL = 0,
    ESP_RTC_EVENT_REGISTERED,
    ESP_RTC_EVENT_INCOMING,
    ESP_RTC_EVENT_CALLING,
    ESP_RTC_EVENT_HANGUP,
    ESP_RTC_EVENT_ERROR,
    ESP_RTC_EVENT_UNREGISTERED,
    ESP_RTC_EVENT_AUDIO_SESSION_BEGIN,
    ESP_RTC_EVENT_AUDIO_SESSION_END,
    ESP_RTC_EVENT_VIDEO_SESSION_BEGIN,
    ESP_RTC_EVENT_VIDEO_SESSION_END,
} esp_rtc_event_t;

typedef int (*esp_rtc_event_handle)(esp_rtc_event_t event);
typedef int (*__esp_rtc_send_audio)(unsigned char *data, int len);
typedef int (*__esp_rtc_receive_audio)(unsigned char *data, int len);
typedef int (*__esp_rtc_send_video)(unsigned char *data, unsigned int *len);
typedef int (*__esp_rtc_receive_video)(unsigned char *data, int len);

typedef struct {
    __esp_rtc_send_audio        send_audio;
    __esp_rtc_receive_audio     receive_audio;
    __esp_rtc_send_video        send_video;
    __esp_rtc_receive_video     receive_video;
} esp_rtc_data_cb_t;

/**
 * @brief ESP RTC video info
 */
typedef struct {
    int     vcodec;     /*!< Video codec type*/
    int     width;      /*!< Video width */
    int     height;     /*!< Video height */
    int     fps;        /*!< Video fps */
    int     len;        /*!< Video len */
} esp_rtc_video_info;

/**
 * @brief RTC session configurations
 */
typedef struct {
    const char                  *uri;               /*!< "Transport://user:pass@server:port/path" */
    rtc_payload_codec_t         aCodecType;         /*!< Audio codec type */
    esp_rtc_video_info          *vCodecInfo;        /*!< Video codec info */
    esp_rtc_data_cb_t           *dataCb;            /*!< RTC data callback */
    esp_rtc_event_handle        eventHandler;       /*!< RTC session event handler */
    bool                        usePublicAddr;      /*!< Use the public IP address returned by the server (RFC3581) */
} esp_rtc_config_t;

/**
 * @brief      Intialize rtc service
 *
 * @param[in]  config       The rtc configuration
 *
 * @return     The rtc handle
 */
esp_rtc_handle_t esp_rtc_init(esp_rtc_config_t *config);

/**
 * @brief      Initialize a rtc session
 *
 * @param[in]  esp_rtc      The rtc handle
 * @param[in]  remote_user  Remote user id
 *
 * @return
 *     - ESP_OK
 *     - ESP_FAIL
 *     - ESP_ERR_INVALID_ARG
 */
int esp_rtc_call(esp_rtc_handle_t esp_rtc, const char *remote_user);

/**
 * @brief      Answer the rtc Session
 *
 * @param[in]  esp_rtc      The rtc handle
 *
 * @return
 *     - ESP_OK
 *     - ESP_FAIL
 *     - ESP_ERR_INVALID_ARG
 */
int esp_rtc_answer(esp_rtc_handle_t esp_rtc);

/**
 * @brief      Hang up or cancel
 *
 * @param[in]  esp_rtc      The rtc handle
 *
 * @return
 *     - ESP_OK
 *     - ESP_FAIL
 *     - ESP_ERR_INVALID_ARG
 */
int esp_rtc_bye(esp_rtc_handle_t esp_rtc);

#ifdef __cplusplus
}
#endif

#endif