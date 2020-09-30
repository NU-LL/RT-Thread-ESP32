/* Examples of speech recognition with multiple keywords.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_log.h"
#include "board.h"
#include "audio_common.h"
#include "audio_pipeline.h"
#include "audio_element.h"
#include "audio_event_iface.h"
#include "audio_mem.h"
#include "mp3_decoder.h"
#include "i2s_stream.h"
#include "raw_stream.h"
#include "filter_resample.h"
#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "dl_lib_coefgetter_if.h"
#include "rec_eng_helper.h"
#include "driver/gpio.h"

#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"


#include <sys/time.h>
#include "sdkconfig.h"
#include "http_stream.h"

#include "esp_peripherals.h"
#include "periph_wifi.h"
#include "esp_http_client.h"
#include "baidu_access_token.h"

static const char *TAG = "asr_keywords";

typedef enum {
    WAKE_UP = 1,
} asr_wakenet_event_t;

typedef enum {
    ID0_DAKAIKONGTIAO       = 0,
    ID1_GUANBIKONGTIAO      = 1,
    ID2_ZENGDAFENGSU        = 2,
    ID3_JIANXIOAFENGSU      = 3,
    ID4_SHENGGAOYIDU        = 4,
    ID5_JIANGDIYIDU         = 5,
    ID6_ZHIREMOSHI          = 6,
    ID7_ZHILENGMOSHI        = 7,
    ID8_SONGFENGMOSHI       = 8,
    ID9_JIENENGMOSHI        = 9,
    ID10_GUANBIJIENENGMOSHI = 10,
    ID11_CHUSHIMOSHI        = 11,
    ID12_GUANBICHUSHIMOSHI  = 12,
    ID13_DAKAILANYA         = 13,
    ID14_GUANBILANYA        = 14,
    ID15_BOFANGGEQU         = 15,
    ID16_ZANTINGBOFANG      = 16,
    ID17_DINGSHIYIXIAOSHI   = 17,
    ID18_DAKAIDIANDENG      = 18,
    ID19_GUANBIDIANDENG     = 19,
    ID20_BANBENHAO          = 20,
    ID21_RUANJIANBAO        = 21,
    ID_MAX,
} asr_multinet_event_t;

#define WAKEUP_TIME_MS 10000

#define GPIO_OUTPUT_IO_0    22
#define GPIO_OUTPUT_IO_1    19
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1))

static esp_err_t asr_multinet_control(int commit_id);
bool enable_multinet = false;




void config_gpio(void)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    // turn off
    gpio_set_level(GPIO_OUTPUT_IO_0, 1);
    gpio_set_level(GPIO_OUTPUT_IO_1, 1);
}


/*
   To embed it in the app binary, the mp3 file is named
   in the component.mk COMPONENT_EMBED_TXTFILES variable.
*/
extern const uint8_t version_mp3_start[] asm("_binary_version_mp3_start");
extern const uint8_t version_mp3_end[]   asm("_binary_version_mp3_end");
extern const uint8_t softpack_mp3_start[] asm("_binary_softpack_mp3_start");
extern const uint8_t softpack_mp3_end[]   asm("_binary_softpack_mp3_end");

// int mp3_pos = 0;
// int mp3_idx = 0;

int version_read_cb(audio_element_handle_t el, char *buf, int len, TickType_t wait_time, void *ctx)
{
    static int mp3_pos;
    int read_size;
    read_size = version_mp3_end - version_mp3_start - mp3_pos;

    if (read_size == 0) {
        mp3_pos = 0;
        return AEL_IO_DONE;
    } else if (len < read_size) {
        read_size = len;
    }
    memcpy(buf, version_mp3_start + mp3_pos, read_size);
    // memcpy(buf, version_mp3_start + mp3_pos, read_size);
    mp3_pos += read_size;
    return read_size;
}

int softpack_read_cb(audio_element_handle_t el, char *buf, int len, TickType_t wait_time, void *ctx)
{
    static int mp3_pos;
    int read_size;
    read_size = softpack_mp3_end - softpack_mp3_start - mp3_pos;

    if (read_size == 0) {
        mp3_pos = 0;
        return AEL_IO_DONE;
    } else if (len < read_size) {
        read_size = len;
    }
    memcpy(buf, softpack_mp3_start + mp3_pos, read_size);
    // memcpy(buf, version_mp3_start + mp3_pos, read_size);
    mp3_pos += read_size;
    return read_size;
}
//0: version
//1: softpack
void play_music(int idx)
{
    audio_pipeline_handle_t pipeline;
    audio_element_handle_t i2s_stream_writer, mp3_decoder;

    // mp3_idx = idx;

    // esp_log_level_set("*", ESP_LOG_WARN);
    // esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "[ 1 ] Start audio codec chip");
    audio_board_handle_t board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);

    ESP_LOGI(TAG, "[ 2 ] Create audio pipeline, add all elements to pipeline, and subscribe pipeline event");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline = audio_pipeline_init(&pipeline_cfg);
    mem_assert(pipeline);

    ESP_LOGI(TAG, "[2.1] Create mp3 decoder to decode mp3 file and set custom read callback");
    mp3_decoder_cfg_t mp3_cfg = DEFAULT_MP3_DECODER_CONFIG();
    mp3_decoder = mp3_decoder_init(&mp3_cfg);
    if(idx == 0)
        audio_element_set_read_cb(mp3_decoder, version_read_cb, NULL);
    else
        audio_element_set_read_cb(mp3_decoder, softpack_read_cb, NULL);

    ESP_LOGI(TAG, "[2.2] Create i2s stream to write data to codec chip");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_WRITER;
    i2s_cfg.i2s_config.sample_rate = 48000;
    i2s_stream_writer = i2s_stream_init(&i2s_cfg);

    ESP_LOGI(TAG, "[2.3] Register all elements to audio pipeline");
    audio_pipeline_register(pipeline, mp3_decoder, "mp3");
    audio_pipeline_register(pipeline, i2s_stream_writer, "i2s1");

    ESP_LOGI(TAG, "[2.4] Link it together [mp3_music_read_cb]-->mp3_decoder-->i2s_stream-->[codec_chip]");

    /**Zl38063 does not support 44.1KHZ frequency, so resample needs to be used to convert files to other rates.
     * You can transfer to 16kHZ or 48kHZ.
     */
#if (CONFIG_ESP_LYRATD_MSC_V2_1_BOARD || CONFIG_ESP_LYRATD_MSC_V2_2_BOARD)
    rsp_filter_cfg_t rsp_cfg = DEFAULT_RESAMPLE_FILTER_CONFIG();
    rsp_cfg.src_rate = 44100;
    rsp_cfg.src_ch = 2;
    rsp_cfg.dest_rate = 48000;
    rsp_cfg.dest_ch = 2;
    audio_element_handle_t filter = rsp_filter_init(&rsp_cfg);
    audio_pipeline_register(pipeline, filter, "filter");
    audio_pipeline_link(pipeline, (const char *[]) {"mp3", "filter", "i2s1"}, 3);
#else
    audio_pipeline_link(pipeline, (const char *[]) {"mp3", "i2s1"}, 2);

#endif
    ESP_LOGI(TAG, "[ 3 ] Set up  event listener");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);

    ESP_LOGI(TAG, "[3.1] Listening event from all elements of pipeline");
    audio_pipeline_set_listener(pipeline, evt);


    ESP_LOGI(TAG, "[ 4 ] Start audio_pipeline");
    audio_pipeline_run(pipeline);
// loop:
    while (1) {
        audio_event_iface_msg_t msg;
        esp_err_t ret = audio_event_iface_listen(evt, &msg, portMAX_DELAY);
        if (ret != ESP_OK) {
            // ESP_LOGE(TAG, "[ * ] Event interface error : %d", ret);
            continue;
        }

        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *) mp3_decoder
            && msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO) {
            audio_element_info_t music_info = {0};
            audio_element_getinfo(mp3_decoder, &music_info);

            ESP_LOGI(TAG, "[ * ] Receive music info from mp3 decoder, sample_rates=%d, bits=%d, ch=%d",
                     music_info.sample_rates, music_info.bits, music_info.channels);

            audio_element_setinfo(i2s_stream_writer, &music_info);

            /* Es8388 and es8374 and es8311 use this function to set I2S and codec to the same frequency as the music file, and zl38063
             * does not need this step because the data has been resampled.*/
#if (CONFIG_ESP_LYRATD_MSC_V2_1_BOARD || CONFIG_ESP_LYRATD_MSC_V2_2_BOARD)
#else
            i2s_stream_set_clk(i2s_stream_writer, music_info.sample_rates , music_info.bits, music_info.channels);
#endif
            continue;
        }
        /* Stop when the last pipeline element (i2s_stream_writer in this case) receives stop event */
        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *) i2s_stream_writer
            && msg.cmd == AEL_MSG_CMD_REPORT_STATUS
            && (((int)msg.data == AEL_STATUS_STATE_STOPPED) || ((int)msg.data == AEL_STATUS_STATE_FINISHED))) {
            break;
        }
    }

    // ESP_LOGI(TAG, "[ 5 ] Stop audio_pipeline");
    // audio_pipeline_terminate(pipeline);

    // audio_element_set_uri(http_stream_reader, BAIDU_TTS_ENDPOINT);
    // mp3_pos = 0;
    // vTaskDelay(10);
    // audio_pipeline_reset_items_state(pipeline);
    // audio_pipeline_reset_ringbuffer(pipeline);
    // audio_pipeline_reset_elements(pipeline);
    // audio_pipeline_change_state(pipeline, AEL_STATE_INIT);
    // audio_pipeline_run(pipeline);
    // goto loop;

    audio_pipeline_unregister(pipeline, mp3_decoder);
    audio_pipeline_unregister(pipeline, i2s_stream_writer);

    /* Terminate the pipeline before removing the listener */
    audio_pipeline_remove_listener(pipeline);

    /* Make sure audio_pipeline_remove_listener is called before destroying event_iface */
    audio_event_iface_destroy(evt);

    /* Release all resources */
    audio_pipeline_unregister(pipeline, i2s_stream_writer);
    audio_pipeline_unregister(pipeline, mp3_decoder);
#if (CONFIG_ESP_LYRATD_MSC_V2_1_BOARD || CONFIG_ESP_LYRATD_MSC_V2_2_BOARD)
    audio_pipeline_unregister(pipeline, filter);
    audio_element_deinit(filter);
#endif
    audio_pipeline_deinit(pipeline);
    // audio_element_deinit(i2s_stream_writer);
    // audio_element_deinit(mp3_decoder);
}



// /* The examples use WiFi configuration that you can set via project configuration menu
//    If you'd rather not, just change the below entries to strings with
//    the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
// */
// #define ESP_WIFI_SSID      "NULL"
// #define ESP_WIFI_PASS      "55555555"
// #define ESP_MAXIMUM_RETRY  5


// #define BAIDU_TTS_ENDPOINT "http://tsn.baidu.com/text2audio"

// #define BAIDU_ACCESS_KEY "riZ91VLHPQxPwEy6jW1jtH8R"
// #define BAIDU_SECRET_KEY "4Emd2xMA7pzuvcpkVzF1IkWrlxjewuLA"

// static char *baidu_access_token = NULL;
// static char request_data[1024];

// const char *tts_text[] = {
//     {" 系统版本号为3.1.3 "},
//     {"实时软件包数量为249"},
//     {"当前运行的线程为"},
//     {"占用线程栈百分之"},
// };

// int tts_text_idx = 0;
// int start_play_flag = 0;

// int _http_stream_event_handle(http_stream_event_msg_t *msg)
// {
//     esp_http_client_handle_t http_client = (esp_http_client_handle_t)msg->http_client;

//     if (msg->event_id != HTTP_STREAM_PRE_REQUEST) {
//         return ESP_OK;
//     }

//     if (baidu_access_token == NULL) {
//         // Must freed `baidu_access_token` after used
//         baidu_access_token = baidu_get_access_token(BAIDU_ACCESS_KEY, BAIDU_SECRET_KEY);
//     }

//     if (baidu_access_token == NULL) {
//         ESP_LOGE(TAG, "Error issuing access token");
//         return ESP_FAIL;
//     }

//     int data_len = snprintf(request_data, 1024, "lan=zh&cuid=ESP32&ctp=1&tok=%s&tex=%s", baidu_access_token, tts_text[tts_text_idx]);
//     esp_http_client_set_post_field(http_client, request_data, data_len);
//     esp_http_client_set_method(http_client, HTTP_METHOD_POST);
//     return ESP_OK;
// }

// esp_periph_set_handle_t set = NULL;
// void init_wifi(void)
// {
//     esp_err_t err = nvs_flash_init();
//     if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
//         // NVS partition was truncated and needs to be erased
//         // Retry nvs_flash_init
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         err = nvs_flash_init();
//     }
//     tcpip_adapter_init();

//     ESP_LOGI(TAG, "[ 0 ] Start and wait for Wi-Fi network");
//     esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
//     set = esp_periph_set_init(&periph_cfg);
//     periph_wifi_cfg_t wifi_cfg = {
//         .disable_auto_reconnect = true,
//         .ssid = ESP_WIFI_SSID,
//         .password = ESP_WIFI_PASS,
//     };
//     esp_periph_handle_t wifi_handle = periph_wifi_init(&wifi_cfg);
//     esp_periph_start(set, wifi_handle);
//     periph_wifi_wait_for_connected(wifi_handle, portMAX_DELAY);
//     ESP_LOGI(TAG, "init Wi-Fi network successfully");
// }

// audio_pipeline_handle_t pipeline;
// audio_element_handle_t http_stream_reader, i2s_stream_writer, mp3_decoder;
// audio_event_iface_handle_t evt = NULL;

// void init_audio_http()
// {
//     // audio_pipeline_handle_t pipeline;
//     // audio_element_handle_t http_stream_reader, i2s_stream_writer, mp3_decoder;

//     ESP_LOGI(TAG, "[ 1 ] Start audio codec chip");
//     audio_board_handle_t board_handle = audio_board_init();
//     audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);

//     ESP_LOGI(TAG, "[2.0] Create audio pipeline for playback");
//     audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
//     pipeline = audio_pipeline_init(&pipeline_cfg);
//     mem_assert(pipeline);

//     ESP_LOGI(TAG, "[2.1] Create http stream to read data");
//     http_stream_cfg_t http_cfg = HTTP_STREAM_CFG_DEFAULT();
//     http_cfg.event_handle = _http_stream_event_handle;
//     http_cfg.type = AUDIO_STREAM_READER;
//     http_stream_reader = http_stream_init(&http_cfg);

//     ESP_LOGI(TAG, "[2.2] Create i2s stream to write data to codec chip");
//     i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
//     i2s_cfg.type = AUDIO_STREAM_WRITER;
//     i2s_stream_writer = i2s_stream_init(&i2s_cfg);


//     ESP_LOGI(TAG, "[2.3] Create mp3 decoder to decode mp3 file");
//     mp3_decoder_cfg_t mp3_cfg = DEFAULT_MP3_DECODER_CONFIG();
//     mp3_decoder = mp3_decoder_init(&mp3_cfg);

//     ESP_LOGI(TAG, "[2.4] Register all elements to audio pipeline");
//     audio_pipeline_register(pipeline, http_stream_reader, "http");
//     audio_pipeline_register(pipeline, mp3_decoder,        "mp3");
//     audio_pipeline_register(pipeline, i2s_stream_writer,  "i2s");

//     ESP_LOGI(TAG, "[2.5] Link it together http_stream-->mp3_decoder-->i2s_stream-->[codec_chip]");
//     audio_pipeline_link(pipeline, (const char *[]) {"http", "mp3", "i2s"}, 3);

//     ESP_LOGI(TAG, "[2.6] Set up  uri (http as http_stream, mp3 as mp3 decoder, and default output is i2s)");
//     audio_element_set_uri(http_stream_reader, BAIDU_TTS_ENDPOINT);

//     ESP_LOGI(TAG, "[ 4 ] Set up  event listener");
//     audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
//     evt = audio_event_iface_init(&evt_cfg);

//     ESP_LOGI(TAG, "[4.1] Listening event from all elements of pipeline");
//     audio_pipeline_set_listener(pipeline, evt);

//     ESP_LOGI(TAG, "[4.2] Listening event from peripherals");
//     audio_event_iface_set_listener(esp_periph_set_get_event_iface(set), evt);
// }

// void play_http_audio(void)
// {
//     static int start_flag = 1;
//     if(start_flag)
//     {
//         start_flag = 0;
//         ESP_LOGI(TAG, "[ 5 ] Start audio_pipeline");
//         audio_pipeline_run(pipeline);
//     }else
//     {
//         audio_element_set_uri(http_stream_reader, BAIDU_TTS_ENDPOINT);
//         audio_pipeline_reset_ringbuffer(pipeline);
//         audio_pipeline_reset_elements(pipeline);
//         // audio_pipeline_change_state(pipeline, AEL_STATE_INIT);
//         audio_pipeline_run(pipeline);
//     }

//     i2s_stream_set_clk(i2s_stream_writer, 16000, 16, 1);
//     while (1) {
//         audio_event_iface_msg_t msg;
//         esp_err_t ret = audio_event_iface_listen(evt, &msg, portMAX_DELAY);
//         if (ret != ESP_OK) {
//             ESP_LOGE(TAG, "[ * ] Event interface error : %d", ret);
//             continue;
//         }

//         if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT
//             && msg.source == (void *) mp3_decoder
//             && msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO) {
//             audio_element_info_t music_info = {0};
//             audio_element_getinfo(mp3_decoder, &music_info);

//             ESP_LOGI(TAG, "[ * ] Receive music info from mp3 decoder, sample_rates=%d, bits=%d, ch=%d",
//                      music_info.sample_rates, music_info.bits, music_info.channels);

//             audio_element_setinfo(i2s_stream_writer, &music_info);
//             i2s_stream_set_clk(i2s_stream_writer, music_info.sample_rates, music_info.bits, music_info.channels);
//             continue;
//         }

//         /* Stop when the last pipeline element (i2s_stream_writer in this case) receives stop event */
//         if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *) i2s_stream_writer
//             && msg.cmd == AEL_MSG_CMD_REPORT_STATUS
//             && (((int)msg.data == AEL_STATUS_STATE_STOPPED) || ((int)msg.data == AEL_STATUS_STATE_FINISHED))) {
//             ESP_LOGW(TAG, "[ * ] Stop event received");
//             break;
//         }
//     }
//     // audio_pipeline_pause(pipeline);
//     ESP_LOGI(TAG, "[ 6 ] Stop audio_pipeline");
//     audio_pipeline_terminate(pipeline);
// }

// void delete_http_audio(void)
// {
//     // ESP_LOGI(TAG, "[ 6 ] Stop audio_pipeline");
//     // audio_pipeline_terminate(pipeline);

//     audio_pipeline_unregister(pipeline, http_stream_reader);
//     audio_pipeline_unregister(pipeline, i2s_stream_writer);
//     audio_pipeline_unregister(pipeline, mp3_decoder);

//     /* Terminal the pipeline before removing the listener */
//     audio_pipeline_remove_listener(pipeline);

//     /* Stop all periph before removing the listener */
//     esp_periph_set_stop_all(set);
//     audio_event_iface_remove_listener(esp_periph_set_get_event_iface(set), evt);

//     /* Make sure audio_pipeline_remove_listener & audio_event_iface_remove_listener are called before destroying event_iface */
//     audio_event_iface_destroy(evt);

//     /* Release all resources */
//     audio_pipeline_deinit(pipeline);
//     audio_element_deinit(http_stream_reader);
//     audio_element_deinit(i2s_stream_writer);
//     audio_element_deinit(mp3_decoder);

//     esp_periph_set_destroy(set);
// }

void tmr_wakeup_cb(xTimerHandle tmr)
{
    ESP_LOGI(TAG, "Wakeup time is out");
    enable_multinet = false;
}

void app_main()
{

#if defined CONFIG_ESP_LYRAT_V4_3_BOARD
    gpio_config_t gpio_conf = {
        .pin_bit_mask = 1UL << get_green_led_gpio(),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = 0
    };
    gpio_config(&gpio_conf);
#endif

    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG, ESP_LOG_INFO);
    // esp_log_level_set("*", ESP_LOG_VERBOSE);
    // esp_log_level_set(TAG, ESP_LOG_VERBOSE);

    config_gpio();

    ESP_LOGI(TAG, "Initialize SR wn handle");
    esp_wn_iface_t *wakenet;
    model_coeff_getter_t *model_coeff_getter;
    model_iface_data_t *model_wn_data;
    const esp_mn_iface_t *multinet = &MULTINET_MODEL;

    get_wakenet_iface(&wakenet);
    get_wakenet_coeff(&model_coeff_getter);
    model_wn_data = wakenet->create(model_coeff_getter, DET_MODE_90);
    int wn_num = wakenet->get_word_num(model_wn_data);
    for (int i = 1; i <= wn_num; i++) {
        char *name = wakenet->get_word_name(model_wn_data, i);
        ESP_LOGI(TAG, "keywords: %s (index = %d)", name, i);
    }
    float wn_threshold = wakenet->get_det_threshold(model_wn_data, 1);
    int wn_sample_rate = wakenet->get_samp_rate(model_wn_data);
    int audio_wn_chunksize = wakenet->get_samp_chunksize(model_wn_data);
    ESP_LOGI(TAG, "keywords_num = %d, threshold = %f, sample_rate = %d, chunksize = %d, sizeof_uint16 = %d", wn_num, wn_threshold, wn_sample_rate, audio_wn_chunksize, sizeof(int16_t));

    ESP_LOGI(TAG, "create");
    model_iface_data_t *model_mn_data = multinet->create(&MULTINET_COEFF, 6000);
    ESP_LOGI(TAG, "get_samp_chunksize");
    int audio_mn_chunksize = multinet->get_samp_chunksize(model_mn_data);
    ESP_LOGI(TAG, "get_samp_chunknum");
    int mn_num = multinet->get_samp_chunknum(model_mn_data);
    ESP_LOGI(TAG, "get_samp_rate");
    int mn_sample_rate = multinet->get_samp_rate(model_mn_data);
    ESP_LOGI(TAG, "keywords_num = %d , sample_rate = %d, chunksize = %d, sizeof_uint16 = %d", mn_num,  mn_sample_rate, audio_mn_chunksize, sizeof(int16_t));

    int size = audio_wn_chunksize;
    if (audio_mn_chunksize > audio_wn_chunksize) {
        size = audio_mn_chunksize;
    }
    int16_t *buffer = (int16_t *)malloc(size * sizeof(short));

    TimerHandle_t tmr_wakeup;
    tmr_wakeup = xTimerCreate("tm_wakeup", WAKEUP_TIME_MS / portTICK_PERIOD_MS, pdFALSE, NULL, tmr_wakeup_cb);

    ESP_LOGI(TAG, "[ 1 ] Start codec chip");
    audio_board_handle_t board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);

    audio_pipeline_handle_t pipeline;
    audio_element_handle_t i2s_stream_reader, filter, raw_read;

    ESP_LOGI(TAG, "[ 2.0 ] Create audio pipeline for recording");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline = audio_pipeline_init(&pipeline_cfg);
    mem_assert(pipeline);

    ESP_LOGI(TAG, "[ 2.1 ] Create i2s stream to read audio data from codec chip");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.i2s_config.sample_rate = 48000;
    i2s_cfg.type = AUDIO_STREAM_READER;

   // Mini board record by I2S1 and play music by I2S0, no need to add resample element.
#if defined CONFIG_ESP_LYRAT_MINI_V1_1_BOARD
    i2s_cfg.i2s_config.sample_rate = 16000;
    i2s_cfg.i2s_port = 1;
    i2s_cfg.i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT;
    i2s_stream_reader = i2s_stream_init(&i2s_cfg);
#else
    i2s_stream_reader = i2s_stream_init(&i2s_cfg);
    ESP_LOGI(TAG, "[ 2.2 ] Create filter to resample audio data");
    rsp_filter_cfg_t rsp_cfg = DEFAULT_RESAMPLE_FILTER_CONFIG();
    rsp_cfg.src_rate = 48000;
    rsp_cfg.src_ch = 2;
    rsp_cfg.dest_rate = 16000;
    rsp_cfg.dest_ch = 1;
    filter = rsp_filter_init(&rsp_cfg);
#endif

    ESP_LOGI(TAG, "[ 2.3 ] Create raw to receive data");
    raw_stream_cfg_t raw_cfg = {
        .out_rb_size = 8 * 1024,
        .type = AUDIO_STREAM_READER,
    };
    raw_read = raw_stream_init(&raw_cfg);

    ESP_LOGI(TAG, "[ 3 ] Register all elements to audio pipeline");
    audio_pipeline_register(pipeline, i2s_stream_reader, "i2s");
    audio_pipeline_register(pipeline, raw_read, "raw");

#if defined CONFIG_ESP_LYRAT_MINI_V1_1_BOARD
    ESP_LOGI(TAG, "[ 4 ] Link elements together [codec_chip]-->i2s_stream-->raw-->[SR]");
    audio_pipeline_link(pipeline, (const char *[]) {"i2s",  "raw"}, 2);
#else
    audio_pipeline_register(pipeline, filter, "filter");
    ESP_LOGI(TAG, "[ 4 ] Link elements together [codec_chip]-->i2s_stream-->filter-->raw-->[SR]");
    audio_pipeline_link(pipeline, (const char *[]) {"i2s", "filter", "raw"}, 3);
#endif

    ESP_LOGI(TAG, "[ 5 ] Start audio_pipeline");
    audio_pipeline_run(pipeline);
    while (1) {
        raw_stream_read(raw_read, (char *)buffer, size * sizeof(short));
        if (wakenet->detect(model_wn_data, (int16_t *)buffer) ==  WAKE_UP) {
            ESP_LOGI(TAG, "wake up");
            enable_multinet = true;
            xTimerStart(tmr_wakeup, portMAX_DELAY);

        }
        if (enable_multinet ==  true) {
            int commit_id = multinet->detect(model_mn_data, buffer);
            if (asr_multinet_control(commit_id) == ESP_OK) {
                enable_multinet = false;
                xTimerStop(tmr_wakeup, portMAX_DELAY);
            }
        }
        vTaskDelay(1);//由于是单核 这里得适当的让出cpu
    }

    ESP_LOGI(TAG, "[ 6 ] Stop audio_pipeline");

    audio_pipeline_terminate(pipeline);

    /* Terminate the pipeline before removing the listener */
    audio_pipeline_remove_listener(pipeline);

    audio_pipeline_unregister(pipeline, raw_read);
    audio_pipeline_unregister(pipeline, i2s_stream_reader);
    audio_pipeline_unregister(pipeline, filter);

    /* Release all resources */
    audio_pipeline_deinit(pipeline);
    audio_element_deinit(raw_read);
    audio_element_deinit(i2s_stream_reader);
    audio_element_deinit(filter);

    ESP_LOGI(TAG, "[ 7 ] Destroy model");
    wakenet->destroy(model_wn_data);
    model_wn_data = NULL;
    free(buffer);
    buffer = NULL;
}

static esp_err_t asr_multinet_control(int commit_id)
{
    if (commit_id >=0 && commit_id < ID_MAX) {
        switch (commit_id) {
            case ID0_DAKAIKONGTIAO:
                ESP_LOGI(TAG, "turn on air-condition");
                break;
            case ID1_GUANBIKONGTIAO:
                ESP_LOGI(TAG, "turn off air-condition");
                break;
            case ID2_ZENGDAFENGSU:
                ESP_LOGI(TAG, "increase in speed");
                break;
            case ID3_JIANXIOAFENGSU:
                ESP_LOGI(TAG, "decrease in speed");
                break;
            case ID4_SHENGGAOYIDU:
                ESP_LOGI(TAG, "increase in temperature");
                break;
            case ID5_JIANGDIYIDU:
                ESP_LOGI(TAG, "decrease in temperature");
                break;
            case ID6_ZHIREMOSHI:
                ESP_LOGI(TAG, "hot mode");
                break;
            case ID7_ZHILENGMOSHI:
                ESP_LOGI(TAG, "slow mode");
                break;
            case ID8_SONGFENGMOSHI:
                ESP_LOGI(TAG, "blower mode");
                break;
            case ID9_JIENENGMOSHI:
                ESP_LOGI(TAG, "save power mode");
                break;
            case ID10_GUANBIJIENENGMOSHI:
                ESP_LOGI(TAG, "turn off save power mode");
                break;
            case ID11_CHUSHIMOSHI:
                ESP_LOGI(TAG, "dampness mode");
                break;
            case ID12_GUANBICHUSHIMOSHI:
                ESP_LOGI(TAG, "turn off dampness mode");
                break;
            case ID13_DAKAILANYA:
                ESP_LOGI(TAG, "turn on bt");
                break;
            case ID14_GUANBILANYA:
                ESP_LOGI(TAG, "turn off bt");
                break;
            case ID15_BOFANGGEQU:
                ESP_LOGI(TAG, "turn on");
                break;
            case ID16_ZANTINGBOFANG:
                ESP_LOGI(TAG, "turn off");
                break;
            case ID17_DINGSHIYIXIAOSHI:
                ESP_LOGI(TAG, "timer one hour");
                break;
            case ID18_DAKAIDIANDENG:
                ESP_LOGI(TAG, "turn on lignt");
                gpio_set_level(GPIO_OUTPUT_IO_0, 0);
                break;
            case ID19_GUANBIDIANDENG:
                ESP_LOGI(TAG, "turn off lignt");
                gpio_set_level(GPIO_OUTPUT_IO_0, 1);
                break;
            case ID20_BANBENHAO:
                ESP_LOGI(TAG, "get RT-Thread version");
                play_music(0);
                break;
            case ID21_RUANJIANBAO:
                ESP_LOGI(TAG, "get RT-Thread softpack number");
                play_music(1);
                break;
            default:
                ESP_LOGI(TAG, "not supportint mode");
                break;
        }
        return ESP_OK;
    }
    return ESP_FAIL;
}
