#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#include "mlx90632.h"
#include "mlx90632_depends.h"

#define MLX90632_SENSOR_ADDR        0x3A                    /*!<  Endereço do sensor no barramento I2C (Endereco escravo)*/
#define VDD_TEMP                    ((1ULL<<I2C_VDD_TEMP))  /*!< Mascara de bits do pino de alimentação do sensor de temperatura */

//Definições de pino I2C
#define I2C_MASTER_SDA_IO           4           /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_SCL_IO           5           /*!< GPIO number used for I2C master clock */
#define I2C_VDD_TEMP                41          /*!< GPIO number used to power the temperature sensor */
#define I2C_MASTER_NUM              I2C_NUM_0   /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          100000      /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000        /*!< I2C master timeout in milliseconds */

/* Function to convert little endian to big endian */
void convert_endiane(uint16_t *data){
    uint16_t temp;
    temp = ((*data << 8) & 0xff00) | (0xff & (*data >> 8));
    *data = temp;
}

/* Function to power the temperature sensor - initialize the VDD_TEMP pin */
static esp_err_t VDD_TEMP_init(){
    gpio_config_t io_conf = {};                     
    
    io_conf.intr_type       = GPIO_INTR_DISABLE; 
    io_conf.mode            = GPIO_MODE_OUTPUT;  
    io_conf.pin_bit_mask    = VDD_TEMP;          
    io_conf.pull_down_en    = 0;                 
    io_conf.pull_up_en      = GPIO_PULLUP_ENABLE;

    gpio_config(&io_conf);
    
    ESP_ERROR_CHECK(gpio_set_level(I2C_VDD_TEMP, 1));
    ESP_LOGI("VDD_TEMP_init", "gpio_set_level executado com sucesso");

    return ESP_OK;
}

/* Function to initialize the I2C master port */
static esp_err_t i2c_master_init(void){
    int i2c_master_port = I2C_MASTER_NUM;
    int pont;

    
    ESP_ERROR_CHECK(VDD_TEMP_init());
    printf("\n");
    ESP_LOGI("i2c_master_init", "Sensor de temperatura ligado");

    i2c_config_t conf = {0};
    
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    ESP_LOGI("i2c_master_init", "Parametro da I2C configurados com sucesso");

    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));
    ESP_LOGI("i2c_master_init", "Driver instalado com sucesso");

    ESP_ERROR_CHECK(i2c_set_timeout(I2C_MASTER_NUM, 19));   
    ESP_ERROR_CHECK(i2c_get_timeout(I2C_MASTER_NUM, &pont));
    //ESP_LOGI("i2c_master_init", "Timeout da I2C = %e", (pont*1.0)/APB_CLK_FREQ);

    return ESP_OK;
}

/* Function to read data from the I2C bus */
esp_err_t i2c_handle_read(uint8_t dev_adr, uint16_t r_adr, size_t r_len, uint8_t *buff){
    
    esp_err_t ret_err = ESP_OK;   
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_start(cmd));                                           //Inicia a comunicação I2C enviando o bit de start 
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_write(cmd, buff, 3, true));                            //envia o endereço do dispositivo e o endereço do registrador que se deseja ler
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_start(cmd));                                           //Reinicia a comunicação I2C para mudar o sentido da comunicação para leitura
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_write_byte(cmd, (dev_adr << 1) | 1, true));            //Envia o endereço do dispositivo com o bit de leitura
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_read(cmd, &buff[1], 1, I2C_MASTER_ACK));               //Recebe o byte mais significativo
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_read(cmd, &buff[0], 1, I2C_MASTER_NACK));              //Recebe o byte menos significativo
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_stop(cmd));                                            //Finaliza a comunicação I2C
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(10000))); //Executa a comunicação I2C

    vTaskDelay(pdMS_TO_TICKS(1000)); //Delay para evitar problemas de comunicação I2C
    i2c_cmd_link_delete(cmd);       //Deleta a estrutura de comando I2C

    return ret_err;
}

/* Function to write data to the I2C bus */
esp_err_t i2c_handle_write(uint8_t dev_adr, uint16_t reg_adr, size_t r_len, uint8_t *buff){
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret_err = ESP_OK;

    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_start(cmd));                       //Inicia a comunicação I2C enviando o bit de start
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_write(cmd, buff, r_len, true));    //envia o endereço do dispositivo e o endereço do registrador que se deseja escrever
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_stop(cmd));                        //Finaliza a comunicação I2C

    ret_err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));   //Executa a comunicação I2C
    if (ret_err != ESP_OK){
        ESP_LOGE("I2C", "i2c_master_cmd_begin failed with error 0x%x", ret_err);
    }

    i2c_cmd_link_delete(cmd);
    return ret_err;
}

int32_t mlx90632_i2c_read32(int16_t register_address, uint32_t *data){
    uint16_t msb_data, lsb_data; // Para armazenar os dados dos registradores MSB e LSB
    int32_t ret;

    // Primeiro, lemos os 16 bits menos significativos
    ret = mlx90632_i2c_read(register_address, &lsb_data);
    if (ret < 0){
        ESP_LOGE("mlx90632_i2c_read32", "Falha na leitura do LSB");
        return ret;
    }    

    // Em seguida, lemos os 16 bits mais significativos
    ret = mlx90632_i2c_read(register_address + 1, &msb_data); // Note o +1 para acessar o próximo registrador
    if (ret < 0){
        ESP_LOGE("mlx90632_i2c_read32", "Falha na leitura do MSB");
        return ret;
    }    

    *data = ((uint32_t)msb_data << 16) | lsb_data; //msb data é convertido para 32 bits antes de ser deslocado para a esquerda por 16 bits e depois é feito um OR com lsb_data para formar um valor de 32 bits
    ret = *data;

    return *data;
}

/* Function to prepare the data to be read on the I2C bus */
int32_t mlx90632_i2c_read(int16_t register_address, uint16_t *data){
    esp_err_t result;
    uint8_t buffer[3]; //Buffer para enviar os dados necessários para a leitura [device addrs, Bit + sign do register_address e Bit - sign do register_address]
    int32_t ret;

    buffer[0] = (MLX90632_SENSOR_ADDR << 1) | 0; //Endereço do dispositivo seguido do bit de escrita (W) igual a 0
    buffer[1] = (register_address >> 8) & 0xFF;  //Bit mais significativo do endereço do registrador
    buffer[2] = register_address & 0xFF;         //Bit menos significativo do endereço do registrador

    result = i2c_handle_read(MLX90632_SENSOR_ADDR, register_address, sizeof(register_address), buffer);
    if(result != ESP_OK){
        ESP_LOGE("mlx90632_i2c_read", "Falha na leitura, código de erro: %d", result);
    }
    
    *data = (buffer[0] << 8) | buffer[1]; //No retorno de (*data) --> buffer[0] contém o byte mais significativo e o buffer[1] contém o byte menos significativo do valor lido

    convert_endiane(data);

    ret = *data & 0x0000FFFF; //retorno do valor lido é igual a 0x0000 + valor lido (16 bits menos significativos)

    return ret;
}

/* Function to prepare the data to be write on the I2C bus */
int32_t mlx90632_i2c_write(int16_t register_address, uint16_t data){
    esp_err_t result;
    uint8_t buffer[5];

    buffer[0] = (MLX90632_SENSOR_ADDR << 1) | 0;
    buffer[1] = (register_address >> 8) & 0xFF; 
    buffer[2] = register_address & 0xFF;        
    buffer[3] = (data >> 8) & 0xFF;             
    buffer[4] = data & 0xFF;                    

    result = i2c_handle_write(MLX90632_SENSOR_ADDR, register_address, 5, buffer);
    if (result != ESP_OK){
        ESP_LOGE("mlx90632_i2c_write", "Falha na leitura, código de erro: %d", result);
    }    

    data = (buffer[3] << 8) | buffer[4];
    return result;
}

static int mlx90632_read_eeprom(int32_t *PR, int32_t *PG, int32_t *PO, int32_t *PT, int32_t *Ea, int32_t *Eb, 
                    int32_t *Fa, int32_t *Fb, int32_t *Ga, int16_t *Gb, int16_t *Ha, int16_t *Hb, int16_t *Ka){

	int32_t ret;
	
    ret = mlx90632_i2c_read(MLX90632_EE_Gb, (uint16_t *) Gb);
	if(ret < 0){
        printf("Erro ao ler o registrador EE_Gb\n");
        return ret;
    }
	ret = mlx90632_i2c_read(MLX90632_EE_Ha, (uint16_t *) Ha);
	if(ret < 0){
        printf("Erro ao ler o registrador EE_Ha\n");
        return ret;
    }
	ret = mlx90632_i2c_read(MLX90632_EE_Hb, (uint16_t *) Hb);
	if(ret < 0){
        printf("Erro ao ler o registrador EE_Hb\n");
        return ret;
    }
	ret = mlx90632_i2c_read(MLX90632_EE_Ka, (uint16_t *) Ka);
	if(ret < 0){
        printf("Erro ao ler o registrador EE_Ka\n");
        return ret;
    }

    ESP_LOGI("mlx90632_read_eeprom", "Registradores de calibração de 16bits lidos com sucesso");

    ret = mlx90632_i2c_read32(MLX90632_EE_P_R, (uint32_t *) PR);
    if(ret < 0){
        printf("Erro ao ler o registrador EE_P_R\n");
        return ret;
    }
	ret = mlx90632_i2c_read32(MLX90632_EE_P_G, (uint32_t *) PG);
    if(ret < 0){
        printf("Erro ao ler o registrador EE_P_G\n");
        return ret;
    }
	ret = mlx90632_i2c_read32(MLX90632_EE_P_O, (uint32_t *) PO);
    if(ret < 0){
        printf("Erro ao ler o registrador EE_P_O\n");
        return ret;
    }
	ret = mlx90632_i2c_read32(MLX90632_EE_P_T, (uint32_t *) PT);
    if(ret < 0){
        printf("Erro ao ler o registrador EE_P_T\n");
        return ret;
    }
	ret = mlx90632_i2c_read32(MLX90632_EE_Ea, (uint32_t *) Ea);
    if(ret < 0){
        printf("Erro ao ler o registrador EE_Ea\n");
        return ret;
    }
	ret = mlx90632_i2c_read32(MLX90632_EE_Eb, (uint32_t *) Eb);
    if(ret < 0){
        printf("Erro ao ler o registrador EE_Eb\n");
        return ret;
    }
	ret = mlx90632_i2c_read32(MLX90632_EE_Fa, (uint32_t *) Fa);
    if(ret < 0){
        printf("Erro ao ler o registrador EE_Fa\n");
        return ret;
    }
	ret = mlx90632_i2c_read32(MLX90632_EE_Fb, (uint32_t *) Fb);
    if(ret < 0){
        printf("Erro ao ler o registrador EE_Fb\n");
        return ret;
    }
	ret = mlx90632_i2c_read32(MLX90632_EE_Ga, (uint32_t *) Ga);
    if(ret < 0){
        printf("Erro ao ler o registrador EE_Ga\n");
        return ret;
    }

    ESP_LOGI("mlx90632_read_eeprom", "Registradores de calibração de 32bits lidos com sucesso");

	return 0;
}


/* Implementation of reading all calibration parameters for calucation of Ta and To */
/*ORIGINAL -- [mlx90632_read_eeprom]
static int mlx90632_read_eeprom(int32_t *PR, int32_t *PG, int32_t *PO, int32_t *PT, int32_t *Ea, int32_t *Eb, 
                    int32_t *Fa, int32_t *Fb, int32_t *Ga, int16_t *Gb, int16_t *Ha, int16_t *Hb, int16_t *Ka){

	int32_t ret;
	
    ret = mlx90632_i2c_read(MLX90632_EE_Gb, (uint16_t *) Gb);
	if(ret < 0){
        printf("Erro ao ler o registrador EE_Gb\n");
        return ret;
    }
	ret = mlx90632_i2c_read(MLX90632_EE_Ha, (uint16_t *) Ha);
	if(ret < 0){
        printf("Erro ao ler o registrador EE_Ha\n");
        return ret;
    }
	ret = mlx90632_i2c_read(MLX90632_EE_Hb, (uint16_t *) Hb);
	if(ret < 0){
        printf("Erro ao ler o registrador EE_Hb\n");
        return ret;
    }
	ret = mlx90632_i2c_read(MLX90632_EE_Ka, (uint16_t *) Ka);
	if(ret < 0){
        printf("Erro ao ler o registrador EE_Ka\n");
        return ret;
    }

    ESP_LOGI("mlx90632_read_eeprom", "Registradores de calibração de 16bits lidos com sucesso");

    //ret = mlx90632_i2c_read32(MLX90632_EE_P_R, (uint32_t *) PR);
    ret = mlx90632_i2c_read(MLX90632_EE_P_R, (uint32_t *) PR);
	if(ret < 0){
        printf("Erro ao ler o registrador EE_P_R\n");
        return ret;
    }
	//ret = mlx90632_i2c_read32(MLX90632_EE_P_G, (uint32_t *) PG);
    ret = mlx90632_i2c_read(MLX90632_EE_P_G, (uint32_t *) PG);
	if(ret < 0){
        printf("Erro ao ler o registrador EE_P_G\n");
        return ret;
    }
	//ret = mlx90632_i2c_read32(MLX90632_EE_P_O, (uint32_t *) PO);
    ret = mlx90632_i2c_read(MLX90632_EE_P_O, (uint32_t *) PO);
	if(ret < 0){
        printf("Erro ao ler o registrador EE_P_O\n");
        return ret;
    }
	//ret = mlx90632_i2c_read32(MLX90632_EE_P_T, (uint32_t *) PT);
    ret = mlx90632_i2c_read(MLX90632_EE_P_T, (uint32_t *) PT);
	if(ret < 0){
        printf("Erro ao ler o registrador EE_P_T\n");
        return ret;
    }
	//ret = mlx90632_i2c_read32(MLX90632_EE_Ea, (uint32_t *) Ea);
    ret = mlx90632_i2c_read(MLX90632_EE_Ea, (uint32_t *) Ea);
	if(ret < 0){
        printf("Erro ao ler o registrador EE_Ea\n");
        return ret;
    }
	//ret = mlx90632_i2c_read32(MLX90632_EE_Eb, (uint32_t *) Eb);
    ret = mlx90632_i2c_read(MLX90632_EE_Eb, (uint32_t *) Eb);
	if(ret < 0){
        printf("Erro ao ler o registrador EE_Eb\n");
        return ret;
    }
	//ret = mlx90632_i2c_read32(MLX90632_EE_Fa, (uint32_t *) Fa);
    ret = mlx90632_i2c_read(MLX90632_EE_Fa, (uint32_t *) Fa);
	if(ret < 0){
        printf("Erro ao ler o registrador EE_Fa\n");
        return ret;
    }
	//ret = mlx90632_i2c_read32(MLX90632_EE_Fb, (uint32_t *) Fb);
    ret = mlx90632_i2c_read(MLX90632_EE_Fb, (uint32_t *) Fb);
	if(ret < 0){
        printf("Erro ao ler o registrador EE_Fb\n");
        return ret;
    }
	//ret = mlx90632_i2c_read32(MLX90632_EE_Ga, (uint32_t *) Ga);
    ret = mlx90632_i2c_read(MLX90632_EE_Ga, (uint32_t *) Ga);
	if(ret < 0){
        printf("Erro ao ler o registrador EE_Ga\n");
        return ret;
    }

    ESP_LOGI("mlx90632_read_eeprom", "Registradores de calibração de 32bits lidos com sucesso");

	return 0;
}*/


/* Main Task of the program */
void teste_init(void *pvParameter){

    int32_t ret = 0;                /**< Variable will store return values */
    double ambient, pre_ambient;    /**< Ambient temperature in degrees Celsius */
    double object, pre_object;      /**< Object temperature in degrees Celsius */ 

    /* Definition of MLX90632 calibration parameters */
	int16_t ambient_new_raw;
	int16_t ambient_old_raw;
	int16_t object_new_raw;
	int16_t object_old_raw;

	int32_t PR = 0x00587f5b;
	int32_t PG = 0x04a10289;
	int32_t PT = 0xfff966f8;
	int32_t PO = 0x00001e0f;
	int32_t Ea = 4859535;
	int32_t Eb = 5686508;
	int32_t Fa = 53855361;
	int32_t Fb = 42874149;
	int32_t Ga = -14556410;
	int16_t Ha = 16384;
	int16_t Hb = 0;
	int16_t Gb = 9728;
	int16_t Ka = 10752;

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(" MAIN", "I2C Mestre iniciado com sucesso");

    ESP_ERROR_CHECK(mlx90632_init());
    ESP_LOGI("MAIN", "Sensor de temperatura iniciado com sucesso");

    /* Put the device in sleeping step mode in order to safely read the EEPROM */
    mlx90632_set_meas_type(MLX90632_MTYP_MEDICAL_BURST);

	/* Read EEPROM calibration parameters */
    ESP_LOGI("MAIN", "Calibrando dispositivo .....");
    mlx90632_read_eeprom(&PR, &PG, &PO, &PT, &Ea, &Eb, &Fa, &Fb, &Ga, &Gb, &Ha, &Hb, &Ka);
	ESP_LOGI("MAIN", "Sensor calibrado com sucesso");

    mlx90632_set_emissivity(0.98); //testar a emissividade de 0.98 (emissividade do ser humano)

    /* Set continuous medical/standard measurement mode */
    if (mlx90632_get_meas_type() != MLX90632_MTYP_MEDICAL){
        mlx90632_set_meas_type(MLX90632_MTYP_MEDICAL);
    }

    while(1){
        printf("\n");
        
		ESP_LOGI("MAIN", "Medindo a temperatura .....");
        ret = mlx90632_read_temp_raw(&ambient_new_raw, &ambient_old_raw, &object_new_raw, &object_old_raw);
        if(ret < 0){
            ESP_LOGE("MAIN", "Erro ao ler a temperatura");
        }

        ambient = mlx90632_calc_temp_ambient(ambient_new_raw, ambient_old_raw, PT, PR, PG, PO, Gb);
        pre_ambient = mlx90632_preprocess_temp_ambient(ambient_new_raw, ambient_old_raw, Gb);
        pre_object = mlx90632_preprocess_temp_object(object_new_raw, object_old_raw, ambient_new_raw, ambient_old_raw, Ka);    
        object = mlx90632_calc_temp_object(pre_object, pre_ambient, Ea, Eb, Ga, Fa, Fb, Ha, Hb);      

		ESP_LOGE("Temperatura", "Ambiente (ºC) = %.2f", ambient);        
        ESP_LOGW("Temperatura", "Objeto (ºC) = %.2f", object);

        vTaskDelay(1000/ portTICK_PERIOD_MS);
    }

    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI("MAIN", "I2C Driver desativado com sucesso");
}

/* Main function of the program -- initialize the Tasks */
int app_main(void){
    ESP_LOGW("MAIN", "Iniciando o programa");
    xTaskCreatePinnedToCore(teste_init,  "teste_init", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);

    return 0;
}