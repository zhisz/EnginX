#include "buzzer.h"

#include "buzzer_config.h"
#include "buzzer_music.h"

static Music_PWM_t *music_handle;
static const TaskHandle_t buzzer_task_handle;

static bool stop = true;

static Music_PWM_t *search_music_from_name(char *name) {
    for (uint8_t i = 0;; i++) {
        if (pwm_musics[i].data == NULL)
            return NULL;

        if (strcmp(name, pwm_musics[i].name) == 0) {
            return (Music_PWM_t *)&pwm_musics[i];
        }
    }
}

// static void buzzer_set_node(uint8_t node_index) {
//     __HAL_TIM_SET_PRESCALER(&BUZZER_TIM, node_pwms[node_index].precaler - 1);
//     __HAL_TIM_SET_AUTORELOAD(&BUZZER_TIM, node_pwms[node_index].counter - 1);

//     __HAL_TIM_SET_COUNTER(&BUZZER_TIM, 0);
//     __HAL_TIM_SET_COMPARE(&BUZZER_TIM, BUZZER_TIM_CHENNEL, node_pwms[node_index].counter / 2);
//     HAL_TIM_PWM_Start(&BUZZER_TIM, BUZZER_TIM_CHENNEL);
// }

static void buzzer_task_t() {
    uint8_t *music = music_handle->data;
    for (uint16_t i = 0;; i += 2) {
        if (music[i] == 0xFF)
            break;

        if (music[i] == ST) {
            continue;
        }

        // buzzer_set_node(music[i]);

        __HAL_TIM_SET_PRESCALER(&BUZZER_TIM, node_pwms[music[i]].precaler - 1);
        __HAL_TIM_SET_AUTORELOAD(&BUZZER_TIM, node_pwms[music[i]].counter - 1);
        __HAL_TIM_SET_COUNTER(&BUZZER_TIM, 0);
        HAL_TIM_PWM_Start(&BUZZER_TIM, BUZZER_TIM_CHENNEL);
        float k = 10.0f;  // 衰减常数，可以调整
        float step;
        uint16_t count;
        if (music[i + 2] == ST) {
            step = 1.0f / ((music[i + 1] + music[i + 3]) * 5);
            count = ((music[i + 1] + music[i + 3]) * 5);
        } else {
            step = 1.0f / (music[i + 1] * 5);
            count = music[i + 1] * 5;
        }

        for (uint16_t j = 0; j < count; j++) {
            if (stop)
                goto buzzer_stop;

            float exp_decay = exp(-k * step * j);  // 指数衰减

            float decay_factor = exp_decay;
            __HAL_TIM_SET_COMPARE(&BUZZER_TIM, BUZZER_TIM_CHENNEL, node_pwms[music[i]].counter * decay_factor);

            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

buzzer_stop:
    stop = true;
    HAL_TIM_PWM_Stop(&BUZZER_TIM, BUZZER_TIM_CHENNEL);
    vTaskDelete(buzzer_task_handle);
}

static void buzzer_play() {
    if (stop) {
        xTaskCreate(buzzer_task_t, "BuzzerPlay", 256, NULL, 254, (TaskHandle_t *const)&buzzer_task_handle);
        stop = false;
    }
}

static void on_bus_call(void *message, BusSubscriberHandle_t subscriber) {
    (void)subscriber;
    char *chose_music = (char *)message;
    music_handle = search_music_from_name(chose_music);
    if (music_handle != NULL)
        buzzer_play();
}

#ifdef BUZZER_USE_CLI
#include <stdio.h>

#include "FreeRTOS_CLI.h"
#include "console_colors.h"
static BaseType_t prvBuzzerCommand(char *pcWriteBuffer,
                                   size_t xWriteBufferLen,
                                   const char *pcCommandString) {
    configASSERT(pcWriteBuffer);
    const char *pcParameter = " ";
    char *param[BUZZER_PARAM_MAX_LEN] = {0};  // 为了分割子命令
    uint8_t param_len = 0;
    BaseType_t xParameterStringLength, xReturn;
    // static UBaseType_t uxParameterNumber = 0;

    while (true) {
        pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, param_len + 1, &xParameterStringLength);
        param[param_len] = (char *)pcParameter;
        if (pcParameter != NULL && *pcParameter != 0)
            param_len++;
        else
            break;
    }

    // 命令参数分割
    for (uint8_t i = 0; param[i] != NULL && i < param_len; i++) {
        char *pc = param[i];
        while (*pc != ' ' && *pc != 0) {
            pc++;
        }
        *pc = 0;
    }

    if (param_len == 1) {
        if (strcmp(param[0], "stop") == 0) {
            stop = true;
        } else
            goto err_use;
    } else if (param_len == 2) {
        if (strcmp(param[0], "play") == 0) {
            if (stop) {
                music_handle = search_music_from_name(param[1]);
                if (music_handle != NULL)
                    buzzer_play();
                else
                    snprintf(pcWriteBuffer, xWriteBufferLen, RED "No Music Found!\n\r" NC);
            } else {
                snprintf(pcWriteBuffer, xWriteBufferLen, RED "Music Playing!\n\r" NC);
            }
        } else
            goto err_use;
    } else
        goto err_use;

    return pdFALSE;

err_use:
    snprintf(pcWriteBuffer, xWriteBufferLen, RED "Wrong Usage!\n\r" NC);
    return pdFALSE;
}
static const CLI_Command_Definition_t xBuzzerCLI = {
    .pcCommand = "buzzer",
    .pcHelpString = "\n\rBuzzer\n\r  Play Buzzer Music!\n\r",
    .cExpectedNumberOfParameters = -1,
    .pxCommandInterpreter = prvBuzzerCommand,
};
#endif

void Bsp_Buzzer_Init() {
    xBusSubscribeFromName("/buzzer", on_bus_call);

    HAL_TIM_PWM_Start(&BUZZER_TIM, BUZZER_TIM_CHENNEL);

#ifdef BUZZER_USE_CLI
    FreeRTOS_CLIRegisterCommand(&xBuzzerCLI);
#endif

}