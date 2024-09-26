#include "esp_idf_version.h"
//#include <stdint.h>
#include <string.h>
//#include <stdbool.h>
#include <stdio.h>
//#include <inttypes.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "math.h"
//#include "time.h"
//#include "sys/time.h"
//#include "esp_private/esp_clk.h"
//#include "driver/mcpwm_cap.h"

#define SPP_TAG "SPP_SERVER"
#define NOME_SERVIDOR_SPP "SPP_SERVER"
#define NOME_SERVIDOR "ESP32_SPP_SERVER"


//-----
static const char *TAG = "ÁPICE";
static const char *ULTRA_TAG = "ULTRA";
static const char *CARRO_TAG  = "CARRO";
static const char *ROBO_TAG  = "ROBO";

#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE        -90   // Minimum angle
#define SERVO_MAX_DEGREE        90    // Maximum angle

#define SERVO_GARRA                  17        // GPIO do servo da garra

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

#define LED 2

int Gatual = -89;

char comando = '0'; //< -- variável que recebe do bluetooth


//---- trabalhando com os eventos do bluetooth

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_NONE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch (event) {
        case ESP_SPP_INIT_EVT:
            ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");

#if ESP_IDF_VERSION <= ESP_IDF_VERSION_VAL(5, 1, 1)

            esp_bt_dev_set_device_name(NOME_SERVIDOR);
#else 
            esp_bt_gap_set_device_name(NOME_SERVIDOR);
#endif

           // esp_bt_dev_set_device_name(NOME_SERVIDOR);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            esp_spp_start_srv(sec_mask, role_slave, 0, NOME_SERVIDOR_SPP);
            break;
        case ESP_SPP_START_EVT:
            ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");
            break;
        case ESP_SPP_DATA_IND_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len=%d handle=%"PRIu32"",
                     param->data_ind.len, param->data_ind.handle);
            esp_log_buffer_hex("", param->data_ind.data, param->data_ind.len);
            comando = param->data_ind.data[0];  //Aqui estamos colocando a letra enviada pelo celular na variável comando <-----------
            ESP_LOGI("APP","%c",comando);
            esp_spp_write(param->data_ind.handle, param->data_ind.len, param->data_ind.data);
            break;
        default:
            break;
    }
}


//----


mcpwm_cmpr_handle_t garra = NULL;



// Função para controle dos servos (não mexer)
static inline uint32_t example_angle_to_compare(int angle)
{
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}


// para controlar cada servo, estes devem ser inicializados, eu inicializei apenas um e deixei um comentário para inicializar outro, lembre-se de que serão 4, portanto adicionar os faltantes
void servo_init()
{
    ESP_LOGI(ROBO_TAG, "Criando o timer e o operador");
    mcpwm_timer_handle_t timerA = NULL;
    mcpwm_timer_handle_t timerB = NULL;

    mcpwm_timer_config_t timer_configA = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    mcpwm_timer_config_t timer_configB = {
        .group_id = 1,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_configA, &timerA));
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_configB, &timerB));

    mcpwm_oper_handle_t operA = NULL;
    mcpwm_oper_handle_t operB = NULL;

    mcpwm_operator_config_t operator_configA = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    mcpwm_operator_config_t operator_configB = {
        .group_id = 1, // operator must be in the same group to the timer
    };

    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_configA, &operA));
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_configB, &operB));

    ESP_LOGI(ROBO_TAG, "Conectando o timer e o operador");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operA, timerA));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operB, timerB));

    ESP_LOGI(ROBO_TAG, "Criando o comparador e gerador");
    
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(operA, &comparator_config, &garra));
    //ESP_ERROR_CHECK(mcpwm_new_comparator(operA, &comparator_config, &horizontal));
    //ESP_ERROR_CHECK(mcpwm_new_comparator(operB, &comparator_config, &erque));
    //ESP_ERROR_CHECK(mcpwm_new_comparator(operB, &comparator_config, &frente));

    mcpwm_gen_handle_t gen1 = NULL;
   // mcpwm_gen_handle_t gen2 = NULL;

    mcpwm_generator_config_t generator_config1 = {.gen_gpio_num = SERVO_GARRA,};
   // mcpwm_generator_config_t generator_config2 = {.gen_gpio_num = SERVO_HORIZ,};

    ESP_ERROR_CHECK(mcpwm_new_generator(operA, &generator_config1, &gen1));
    //ESP_ERROR_CHECK(mcpwm_new_generator(operA, &generator_config2, &gen2));


    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(garra, example_angle_to_compare(-89)));   //< Posição inicial da garra 
    //ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(horizontal, example_angle_to_compare(0)));    

    ESP_LOGI(ROBO_TAG, "Configurando gerador");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen1,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    //ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen2,
    //                MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen1,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, garra, MCPWM_GEN_ACTION_LOW)));
    //ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen2,
    //                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, horizontal, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(ROBO_TAG, "Habilitando o timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timerA));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timerA, MCPWM_TIMER_START_NO_STOP));
    ESP_ERROR_CHECK(mcpwm_timer_enable(timerB));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timerB, MCPWM_TIMER_START_NO_STOP));
    

    
}

const double pi = 3.1415;


void servobot_modif (int G, int H, int E, int F) 
{
    int posicao_garra;
    int Gdif = Gatual;
   // int Hdif = Hatual;
   // int Fdif = Fatual;
   // int Edif = Eatual;

    if(G > 0 && Gatual > -89) {Gdif = Gatual - 1;}  //-89
    else if(G < 0 && Gatual < -49) {Gdif = Gatual + 1;} //-49
   /* if(H > 0 && Hatual > -89) {Hdif = Hatual - 1;} //-89
    else if(H < 0 && Hatual < 89) {Hdif = Hatual + 1;} //89
    if(F > 0 && Fatual > -89) {Fdif = Fatual - 1;} //-89
    else if(F < 0 && Fatual < 44) {Fdif = Fatual + 1;} //0  ~44
    if(E > 0 && Eatual > -89) {Edif = Eatual - 1;} //-50
    else if(E < 0 && Eatual < 30) {Edif = Eatual + 1;} //44 ~30*/

   // ESP_LOGI(ROBO_TAG,"Gnovo %d - Hnovo %d - Fnovo %d - Enovo %d", Gdif, Hdif, Fdif, Edif);

    posicao_garra = Gdif;
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(garra, example_angle_to_compare(posicao_garra)));
  //  posicao_horiz = Hdif;
  //  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(horizontal, example_angle_to_compare(posicao_horiz)));

    Gatual = posicao_garra;
  //  Hatual = p.posicao_horiz;

  //  ESP_LOGI(ROBO_TAG,"Garra %d - Horizontal %d - Frontal %d - Vertical %d", Gatual, Hatual, Fatual, Eatual);

}

void carro_frente()
{
   /* gpio_set_level(CARRO_M1A,1);
    gpio_set_level(CARRO_M1B,0);
    gpio_set_level(CARRO_M2A,1);
    gpio_set_level(CARRO_M2B,0);*/
}

void pisca(int vezes, int tempo)
{
    for (int i = 0; i < vezes; i++)
    {      
        gpio_set_level(LED,1);
        vTaskDelay(pdMS_TO_TICKS(tempo));
        gpio_set_level(LED,0);
        vTaskDelay(pdMS_TO_TICKS(tempo));
    }
}



void init_bt()
{
    //--- configurações do bluetooth (Não mexer)
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret) {
        ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(SPP_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(SPP_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_spp_register_callback(esp_spp_cb);
    if (ret) {
        ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ESP_ERROR_CHECK(esp_spp_init(esp_spp_mode));
    //ESP_ERROR_CHECK(esp_spp_init(esp_spp_mode));

    ESP_LOGI(SPP_TAG, "Bluetooth initialized, waiting for connections...");
    // ok configuração do bluetooth finalizada
}


void init_io()
{
  /*  gpio_reset_pin(CARRO_M1A);
    gpio_set_direction(CARRO_M1A,GPIO_MODE_OUTPUT);
    gpio_set_level(CARRO_M1A,0);*/
}


void app_main(void)
{    
    init_io();
    init_bt();
    servo_init();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10)); //delay mínimo dentro do while

        switch (comando)
        {
            case 'A':    servobot_modif(-1,0,0,0); ESP_LOGI(TAG,"Recebido comando A") ; break; //ABRE - Coloca a posição da garra em aberto
        //  case 'B':                               break;
        //  case 'C':                               break;
        //  ...
            default:                                break; //LIVRE
        }
    }
}

