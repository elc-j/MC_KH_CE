/**
  ******************************************************************************
  * @file    master_app.c
  * @author  MCD Application Team
  * @brief   Application of the AT Master
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "sys_app.h"
#include "master_app.h"
#include "lora_driver.h"
#include "sys_sensors.h"
#include "rtc_if.h"
#include "stdio.h"
#include "lrwan_ns1_printf.h"



#include ATCMD_MODEM        /* preprocessing definition in sys_conf.h*/



/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/


/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* CAYENNE_LLP is myDevices Application server*/
#define CAYENNE_LPP
#define LPP_DATATYPE_DIGITAL_INPUT 0x0
#define LPP_DATATYPE_DIGITAL_INPUT_SIZE 1
#define LPP_DATATYPE_HUMIDITY 0x68
#define LPP_DATATYPE_HUMIDITY_SIZE 1        // humidity sensor data size
#define LPP_DATATYPE_TEMPERATURE 0x67
#define LPP_DATATYPE_TEMPERATURE_SIZE 2
#define LPP_DATATYPE_BAROMETER 0x73
#define LPP_DATATYPE_BAROMETER_SIZE 2
#define LPP_DATATYPE_FRAME_IDENTIFIER 0x99  // identifier of data frame (who is it coming from)

/* Private variables ---------------------------------------------------------*/

static sensor_t Sensor;                            /* struct for data sensor*/

#if USE_LRWAN_NS1
static bool user_button_flag = true;
#endif


/* Private function prototypes -----------------------------------------------*/

static void SensorMeasureData(sSendDataBinary_t *SendDataBinary);

/* load call backs*/
static LoRaDriverCallback_t LoRaDriverCallbacks = { SensorMeasureData,
                                                    NULL
                                                  };

#if USE_LRWAN_NS1
/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected to the EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /*HW_RTC_DelayMs(10);*/
  RTC_IF_DelayMs(10);
  if (!HAL_GPIO_ReadPin(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN))
  {
    if (user_button_flag)
    {
      Modem_IO_DeInit();
      BSP_LED_DeInit(LED2);
      user_button_flag = false;
      while (1);
    }
    else
    {
      Modem_IO_Init();
      NVIC_SystemReset();
    }
  }
}
#endif

/* Private macro ------------- -----------------------------------------------*/

#ifdef CAYENNE_LPP
#define LORAWAN_APP_PORT           99;            /*LoRaWAN application port*/
#else
#define LORAWAN_APP_PORT           2;            /*LoRaWAN application port*/
#endif

#define LORAWAN_CONFIRMED_MSG      ENABLE         /*LoRaWAN confirmed messages*/

#define SENSORS_MEASURE_CYCLE      15000          /*Periode to do sensors measurement*/

#define JOIN_MODE                  OTAA_JOIN_MODE   /*ABP_JOIN_MODE */ /*LoRaWan join methode*/

#ifdef USE_LRWAN_NS1
#define FREQ_BAND                  /*EU868*/ /*CN470PREQUEL*/ EU433 // YB: move from CN470PREQUEL to EU433 
#endif


/* Init LoRa Driver modem parameters*/
#ifdef USE_LRWAN_NS1
static LoRaDriverParam_t LoRaDriverParam = {  SENSORS_MEASURE_CYCLE,  JOIN_MODE, FREQ_BAND};
#else
static LoRaDriverParam_t LoRaDriverParam = {  SENSORS_MEASURE_CYCLE,  JOIN_MODE};
#endif



/*!
  * Master context initialization following the LoRa modem used
  *
  */
void MasterApp_Init(void)
{

  /* if using sequencer uncomment the task creation */
  /*UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_Lora_fsm), UTIL_SEQ_RFU, Lora_fsm); */

  Lora_Ctx_Init(&LoRaDriverCallbacks, &LoRaDriverParam);
}

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/********************* LoRa Part Apllication **********************************/

/******************************************************************************
  * @brief SensorMeasureData
  * @param none
  * @return none
  ******************************************************************************/
static void SensorMeasureData(sSendDataBinary_t *SendDataBinary)
{
#ifndef CAYENNE_LPP
  uint8_t LedState = 0;                /*just for padding*/
#endif
  // 1. TODO LORA USE_LRWAN_NS1: uncomment those variables below vvv

  float pressure = 0;
  float temperature = 0;
  float humidity = 0;

  int16_t temperature_int = 0;
  uint8_t humidity_int = 0;
  uint16_t pressure_int = 0;

 uint32_t BatLevel = 0;               // end device connected to external power source
 ATEerror_t LoraCmdRetCode;

  uint8_t index = 0;
  /*read pressure, Humidity and Temperature in order to be send on LoRaWAN*/
  EnvSensors_Read(&Sensor);


#ifdef CAYENNE_LPP
  // 1. TODO LORA USE_LRWAN_NS1: uncomment this variable below vvv too
//uint8_t channel = 0;
  pressure = Sensor.pressure;
  temperature = Sensor.temperature;
  humidity = Sensor.humidity;

  // 1. TODO LORA USE_LRWAN_NS1: THEN: print pressure, temperature, humidity on terminal!
  // 1. TODO LORA USE_LRWAN_NS1: with two decimals precision
  // 1. TODO LORA USE_LRWAN_NS1: #if defined()/#endif style
  // 1. TODO LORA USE_LRWAN_NS1: hint: use dbg_printf_send() (where is it? how does it work?)
   #if defined(USE_LRWAN_NS1)
   dbg_printf_send("Temperature_float = %.2f C\n", temperature);
   dbg_printf_send("Humidity_float = %.2f % \n", humidity);
   dbg_printf_send("Pressure_float = %.2f hPa\n", pressure);
   #endif


  // 6. TODO LORA: convert temperature, pressure, humidity to data for SendDataBinary->Buffer
  // 6. TODO LORA: hint: decidegrees, decahPas, double humidity percents
  // 6. TODO LORA: do the proper final type casts for each of those values!
   temperature_int = (int16_t)(Sensor.temperature * 10.0f);
   dbg_printf_send("Temperature_int = %d C\n", temperature_int);

   humidity_int = (uint8_t)(Sensor.humidity * 0.5f);
   dbg_printf_send("Humidity_int = %d %\n", humidity_int);

   pressure_int = (uint16_t)(Sensor.pressure * 10.0f);
   dbg_printf_send("Pressure_int = %d daPa\n", pressure_int);
 
  // 7. TODO LORA: create your data payload as per defined in the practical work
  // 7. TODO LORA: you will write into the SendDataBinary->Buffer

   // ----------------------------------  CODE TRAME ------------------------------------
   uint8_t block = 1;

     /* ---------- Block 1 : FRAME IDENTIFIER  ---------- */

     const char group_id[] = "BW_CM_HK_EC";
     uint8_t gid_len = (uint8_t)strlen(group_id);

     SendDataBinary->Buffer[index++] = block++;
     SendDataBinary->Buffer[index++] = LPP_DATATYPE_FRAME_IDENTIFIER;  // 0x99
     SendDataBinary->Buffer[index++] = gid_len;

     for (int i = 0; i < gid_len; i++) {
       SendDataBinary->Buffer[index++] = (uint8_t)group_id[i];
     }


     /* ---------- Block 2 : PRESSURE ---------- */

     SendDataBinary->Buffer[index++] = block++;
     SendDataBinary->Buffer[index++] = LPP_DATATYPE_BAROMETER;       // 0x73
     SendDataBinary->Buffer[index++] = LPP_DATATYPE_BAROMETER_SIZE;  // 2
     SendDataBinary->Buffer[index++] = (pressure_int >> 8) & 0xFF;          // MSB
     SendDataBinary->Buffer[index++] = pressure_int & 0xFF;                 // LSB

     /* ---------- Block 3 : TEMPERATURE---------- */
     SendDataBinary->Buffer[index++] = block++;
     SendDataBinary->Buffer[index++] = LPP_DATATYPE_TEMPERATURE;       // 0x67
     SendDataBinary->Buffer[index++] = LPP_DATATYPE_TEMPERATURE_SIZE;  // 2
     SendDataBinary->Buffer[index++] = (temperature_int >> 8) & 0xFF;          // MSB
     SendDataBinary->Buffer[index++] = temperature_int & 0xFF;                 // LSB

     /* ---------- Block 4 : HUMIDITY---------- */

     SendDataBinary->Buffer[index++] = block++;
     SendDataBinary->Buffer[index++] = LPP_DATATYPE_HUMIDITY;       // 0x68
     SendDataBinary->Buffer[index++] = LPP_DATATYPE_HUMIDITY_SIZE;  // 2
     SendDataBinary->Buffer[index++] = humidity_int;

     /* ---------- Block 5 : BATTERIE  ---------- */

    LoraCmdRetCode = Lora_GetBatLevel(&BatLevel);

   #if defined(USE_LRWAN_NS1)
     if (LoraCmdRetCode != ATCTL_RET_CMD_VDD)
     {
    	 BatLevel = 0;
     }
   #else
     if (LoraCmdRetCode != AT_OK)
     {
    	 BatLevel = 0;
     }
   #endif

     uint8_t bat_pct;

     if (BatLevel >= 254U) {
       bat_pct = 100U;
     } else {
       bat_pct = (uint8_t)((BatLevel * 100U) / 254U);
     }

     SendDataBinary->Buffer[index++] = block++;
     SendDataBinary->Buffer[index++] = LPP_DATATYPE_DIGITAL_INPUT;      // 0x00
     SendDataBinary->Buffer[index++] = LPP_DATATYPE_DIGITAL_INPUT_SIZE; // 1
     SendDataBinary->Buffer[index++] = bat_pct;

  // ---- Vérification -----------

     dbg_printf_send("Payload size=%d\r\n", index);
     for (uint8_t i = 0; i < index; i++)
     {
       dbg_printf_send("%02X ", (uint8_t)SendDataBinary->Buffer[i]);
     }
     dbg_printf_send("\r\n");

 //--------------------------------    FIN CODE TRAME -----------------------------------------

#else

  int32_t latitude, longitude = 0;     /*just for padding*/
  uint16_t altitudeGps = 0;            /*just for padding*/

  temperature = 234; // YB: example decidgrees
  pressure = 11111;  // YB: example daPascals
  humidity = 750;    // YB: example decipercents of humidity, but we want double of percents instead!

  latitude = Sensor.latitude;    /* not relevant*/
  longitude = Sensor.longitude;  /* not relevant*/

  /*fill up the send buffer*/
  SendDataBinary->Buffer[index++] = LedState;
  SendDataBinary->Buffer[index++] = (pressure >> 8) & 0xFF;
  SendDataBinary->Buffer[index++] =  pressure & 0xFF;
  SendDataBinary->Buffer[index++] = (temperature >> 8) & 0xFF;
  SendDataBinary->Buffer[index++] =  temperature & 0xFF;
  SendDataBinary->Buffer[index++] = (humidity >> 8) & 0xFF;
  SendDataBinary->Buffer[index++] =  humidity & 0xFF;

  /*get battery level of the modem (slave)*/
  LoraCmdRetCode = Lora_GetBatLevel(&BatLevel);
  if (LoraCmdRetCode == AT_OK)
  {
    DBG_PRINTF("Msg status = %d --> OK\n", BatLevel);
  }
  else
  {
    DBG_PRINTF("Msg status = %d --> KO\n", BatLevel);
  }
  SendDataBinary->Buffer[index++] = (uint8_t)BatLevel;

  /*remaining data just for padding*/
  SendDataBinary->Buffer[index++] = (latitude >> 16) & 0xFF;
  SendDataBinary->Buffer[index++] = (latitude >> 8) & 0xFF;
  SendDataBinary->Buffer[index++] = latitude & 0xFF;
  SendDataBinary->Buffer[index++] = (longitude >> 16) & 0xFF;
  SendDataBinary->Buffer[index++] = (longitude >> 8) & 0xFF;
  SendDataBinary->Buffer[index++] = longitude & 0xFF;
  SendDataBinary->Buffer[index++] = (altitudeGps >> 8) & 0xFF;
  SendDataBinary->Buffer[index++] = altitudeGps & 0xFF;

#endif  /*CAYENNE_LPP*/

  // 8. TODO LORA: This is the end of your custom data. Leave the rest below untouched 

  SendDataBinary->DataSize = index;
#ifndef USE_LRWAN_NS1
  SendDataBinary->Port = LORAWAN_APP_PORT;
#endif

#ifdef USE_I_NUCLEO_LRWAN1
  SendDataBinary->Ack = !LORAWAN_CONFIRMED_MSG;
#endif
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t *file, uint32_t line)
{

  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

}

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
