/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c_master.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MODBUS_RTU_ID 1															// Endereço de Escravo do STM32
#define MODBUS_RTU_Timeout 1000													// Em ms
#define MODBUS_RTU_Broadcast 0													// 0 - Desativado | 1 - Ativado
#define MODBUS_RTU_USART huart6													// USART selecionada, caso troque pela interface IOC, atualizar manualmente por aqui.
#define MODBUS_RTU_HDMA_USART_RX hdma_usart6_rx									// USART selecionada, caso troque pela interface IOC, atualizar manualmente por aqui.
#define MODBUS_RTU_Tamanho_Buffer_RX 256										// Tamanho máximo para evitar sobrecarga

#define MODBUS_RTU_Codigo_Read_Coil 1
#define MODBUS_RTU_Codigo_Read_Discrete_Input 2
#define MODBUS_RTU_Codigo_Read_Holding_Register 3
#define MODBUS_RTU_Codigo_Read_Input_Register 4
#define MODBUS_RTU_Codigo_Write_Single_Coil 5
#define MODBUS_RTU_Codigo_Write_Single_Register 6
#define MODBUS_RTU_Codigo_Write_Multiple_Coils 15
#define MODBUS_RTU_Codigo_Write_Multiple_Registers 16

#define MODBUS_RTU_Codigo_Erro_Read_Coil 129
#define MODBUS_RTU_Codigo_Erro_Read_Discrete_Input 130
#define MODBUS_RTU_Codigo_Erro_Read_Holding_Register 131
#define MODBUS_RTU_Codigo_Erro_Read_Input_Register 132
#define MODBUS_RTU_Codigo_Erro_Write_Single_Coil 133
#define MODBUS_RTU_Codigo_Erro_Write_Single_Register 134
#define MODBUS_RTU_Codigo_Erro_Write_Multiple_Coils 143
#define MODBUS_RTU_Codigo_Erro_Write_Multiple_Registers 144

#define MODBUS_RTU_Codigo_Erro_Illegal_Function 1								// Código de função recebido na consulta não é reconhecido ou permitido pelo servidor
#define MODBUS_RTU_Codigo_Erro_Illegal_Data_Address	2							// O endereço de dados de algumas ou todas as entidades necessárias não é permitido ou não existe no servidor
#define MODBUS_RTU_Codigo_Erro_Illegal_Data_Value 3								// O valor não é aceito pelo servidor
#define MODBUS_RTU_Codigo_Erro_Server_Device_Failure 4							// Ocorreu um erro irrecuperável enquanto o servidor tentava executar a ação solicitada
#define MODBUS_RTU_Codigo_Erro_Acknowledge 5									// O servidor aceitou a solicitação e está processando-a, mas é necessário um longo período de tempo. Essa resposta é retornada para evitar que ocorra um erro de tempo limite no cliente. Em seguida, o cliente pode emitir uma mensagem de Poll Program Complete para determinar se o processamento foi concluído
#define MODBUS_RTU_Codigo_Erro_Server_Device_Busy 6								// O servidor está envolvido no processamento de um comando de longa duração; o cliente deve tentar novamente mais tarde
#define MODBUS_RTU_Codigo_Erro_Negative_Acknowledge	7							// O servidor não pode executar as funções de programação; o cliente deve solicitar informações de diagnóstico ou erro do servidor
#define MODBUS_RTU_Codigo_Erro_Memory_Parity_Error 8							// O servidor detectou um erro de paridade na memória; o cliente pode repetir a solicitação
#define MODBUS_RTU_Codigo_Erro_Gateway_Path_Unavailable	10						// Especializado para gateways Modbus: indica um gateway mal configurado
#define MODBUS_RTU_Codigo_Erro_Gateway_Target_Device_Failed_to_Respond 11		// Especializado para gateways Modbus: enviado quando o servidor não responde

#define GPIO_LED_BP GPIOC
#define GPIO_LED_Erro GPIOB

#define TempoTesteReles 100
#define TEMPO_LED_ERRO 10

#define QTD_ERRO_MODBUS 10

#define USART6_PERSONALIZADO 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define	Bit(n)							(1 << n) 										// retorna o valor de um bit específico
#define CLEAR_Bit(x, Bit)				(x &= ~(1 << Bit))								// limpa o valor de um bit específico em uma variável
#define GET_Bit(x, n)   				(((x) >> (n)) & 0x01)							// retorna o valor de um bit específico em uma variável									// retorna o valor de um bit específico em uma variável
#define SET_Bit(x, Bit)					(x |= (1 << Bit))								// define o valor de um bit específico em uma variável
#define TEST_Bit(x, Bit)				((x >> Bit) & 1)								// retorna se o valor de um bit específico em uma variável é 1 ou 0
#define TOGGLE_Bit(x, Bit)				(x ^= (1 << Bit))								// inverte o valor de um bit específico em uma variável
#define ROTATE_LEFT(x, n)				(x = (x << n) | (x >> (sizeof(x) * 8 - n)))		// rotaciona os bits de uma variável x para a esquerda n vezes
#define ROTATE_RIGHT(x, n)				(x = (x >> n)									// rotaciona os bits de uma variável x para a direita n vezes
#define compare_enum_greater_equal(x,y) ((uint16_t)(x)>=(uint16_t)(y))
#define MIN(a, b) 						(((a) < (b)) ? (a) : (b))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */
struct _Bits
{
	unsigned WD_MODBUS_RTU : 1;
	unsigned M_Baud_9600 : 1;
	unsigned M_Baud_19200 : 1;
	unsigned M_Teste_Reles : 1;
	unsigned M_LED_Erro : 1;
	unsigned M_Troca_Baud : 1;
}Bit;

uint8_t txDataValue[2] = {0};
uint8_t rxDataValue[2] = {0};

volatile uint16_t MODBUS_CRC;					//CRC Total
volatile uint8_t MODBUS_CRC_High_Byte;			//CRC High Byte
volatile uint8_t MODBUS_CRC_Low_Byte;			//CRC Low Byte

volatile uint8_t MODBUS_RTU_Escravo = MODBUS_RTU_ID;

uint8_t MODBUS_RTU_Buffer_RX[256]; //Variável Global Para Recebimento de Dados Serial.
uint8_t MODBUS_RTU_Buffer_TX[256]; //Variável Global Para Transmissão de Dados Serial.

uint8_t MODBUS_RTU_Buffer_RX_Auxiliar[256]; //Variável Global Para Recebimento de Dados Serial.

//uint8_t MODBUS_RTU_Tamanho_Buffer_RX; //Variável Global Para Tamanho do RX.

volatile uint16_t MODBUS_RTU_Endereco = 0;
volatile uint16_t MODBUS_RTU_Quantidade_Bits = 0;
volatile uint16_t MODBUS_RTU_Quantidade_Bytes = 0;
volatile uint16_t MODBUS_RTU_Quantidade_Registers = 0;

volatile uint8_t MODBUS_RTU_Valor_Bit = 0;
volatile uint16_t MODBUS_RTU_Valor_Registrador = 0;

uint8_t LED = 0;

uint8_t flagI2C = 0;

volatile uint8_t ContadorErroModbus = 0;
volatile uint8_t ContadorInterrupcaoUsartModbus = 0;
volatile uint8_t TemporizadorInterrupcaoUsartModbus = 0;

volatile uint16_t ContadorTesteReles = 0;
volatile uint16_t ContadorLedErro = 0;

volatile uint32_t baudRate = 9600;
// Variáveis relativas ao Modbus

// Declaração dos endereços do tipo discrete inputs (bits somente de leitura)
enum _Enderecos_Discrete_Inputs
{
	Alarme1,
	Alarme2,
	Alarme3,
	Alarme4,
	Alarme5,
	Alarme6,
	Alarme7,
	Alarme8,
	Alarme9,
	Alarme10,
	Alarme11,
	Alarme12,

	Quantidade_Discrete_Inputs = 16
}Enderecos_Discrete_Inputs;

// Declaração dos endereços do tipo coils (bits de leitura e escrita)
enum _Enderecos_Coils
{

	Quantidade_Coils = 16
}Enderecos_Coils;

// Declaração dos endereços do tipo input registers (registradores somente de leitura)
enum _Enderecos_Input_Registers
{
	Alarmesx1,
	Alarmesx2,

	Quantidade_Input_Registers
}Enderecos_Input_Registers;

// Declaração dos endereços do tipo holding registers (registradores de leitura e escrita)
enum _Enderecos_Holding_Registers
{

	Quantidade_Holding_Registers
}Enderecos_Holding_Registers;

// Matrizes do Modbus

// Declaração de uma matriz para armazenar os valores referentes às discrete inputs
uint8_t Discrete_Inputs[Quantidade_Discrete_Inputs];

// Declaração de uma matriz para armazenar os valores referentes aos coils
uint8_t Coils[Quantidade_Coils];

// Declaração de uma matriz para armazenar os valores referentes aos input registers
uint16_t Input_Registers[Quantidade_Input_Registers];

// Declaração de uma matriz para armazenar os valores referentes aos holding registers
uint16_t Holding_Registers[Quantidade_Holding_Registers];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
//Função de Cálculo CRC Modbus;
uint16_t Calculo_CRC_MODBUS(uint8_t buf[], uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= buf[i];

        for (uint16_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else crc = (crc >> 1);
        }
    }
    return crc;
}

void MODBUS_RTU_Funcao_Erro(uint8_t Codigo_Erro)		// FUNÇÃO ERRO
{
	if (MODBUS_RTU_Buffer_RX[0] != 0)
	{
		MODBUS_RTU_Buffer_TX[0] = MODBUS_RTU_Escravo;								//Endereço de Escravo
		MODBUS_RTU_Buffer_TX[1] = MODBUS_RTU_Buffer_RX[1] + 128;					//Código da Função com Erro
		MODBUS_RTU_Buffer_TX[2] = Codigo_Erro;										//

		MODBUS_CRC = Calculo_CRC_MODBUS(MODBUS_RTU_Buffer_TX, 3);

		MODBUS_RTU_Buffer_TX[3] = MODBUS_CRC & 0xFF;
		MODBUS_RTU_Buffer_TX[4] = MODBUS_CRC >> 8;

		HAL_UART_Transmit(&MODBUS_RTU_USART, MODBUS_RTU_Buffer_TX, 5, MODBUS_RTU_Timeout);

		if (Bit.M_LED_Erro == 0)
		{
			HAL_GPIO_WritePin(GPIO_LED_Erro, LED_Erro_Pin, 1);
			Bit.M_LED_Erro = 1;
		}

		ContadorErroModbus = ContadorErroModbus + 2;
	}
}

void MODBUS_RTU_Funcao_Read_Coil()						// FUNÇÃO 01
{
	if (Quantidade_Coils > 0)
	{
		MODBUS_RTU_Endereco = (MODBUS_RTU_Buffer_RX[2] << 8) | (MODBUS_RTU_Buffer_RX[3]);
		MODBUS_RTU_Quantidade_Bits = (MODBUS_RTU_Buffer_RX[4] << 8) | (MODBUS_RTU_Buffer_RX[5]);
		MODBUS_RTU_Quantidade_Bytes = (MODBUS_RTU_Quantidade_Bits + 7) / 8;

		if ((MODBUS_RTU_Endereco + MODBUS_RTU_Quantidade_Bits) <= MIN(Quantidade_Coils, 9999))
		{
			MODBUS_RTU_Buffer_TX[0] = MODBUS_RTU_Escravo;												//Endereço de Escravo
			MODBUS_RTU_Buffer_TX[1] = MODBUS_RTU_Codigo_Read_Coil;										//Código da Função
			MODBUS_RTU_Buffer_TX[2] = MODBUS_RTU_Quantidade_Bytes;										//Número de Bytes de Dados

			uint8_t Contador = 0;
			for (int i = 0; i < MODBUS_RTU_Quantidade_Bits; i++)
			{
			    if (Contador < 8)
			    {
			    	if (Coils[i + MODBUS_RTU_Endereco] == 1)
			    	{
			    		SET_Bit (MODBUS_RTU_Buffer_TX[3 + (i / 8)], Contador);
			    	}
			    	else
			    	{
			    		CLEAR_Bit (MODBUS_RTU_Buffer_TX[3 + (i / 8)], Contador);
			    	}
			        Contador++;
			    }
			    if (Contador >= 8) Contador = 0;
			}

			//Executa Função do Cálculo do CRC
			MODBUS_CRC = Calculo_CRC_MODBUS(MODBUS_RTU_Buffer_TX, MODBUS_RTU_Quantidade_Bytes + 3);
			MODBUS_RTU_Buffer_TX[MODBUS_RTU_Quantidade_Bytes + 3] = MODBUS_CRC & 0xFF;
			MODBUS_RTU_Buffer_TX[MODBUS_RTU_Quantidade_Bytes + 4] = MODBUS_CRC >> 8;

			HAL_UART_Transmit(&MODBUS_RTU_USART, MODBUS_RTU_Buffer_TX, MODBUS_RTU_Quantidade_Bytes + 5, MODBUS_RTU_Timeout);
		}
		else MODBUS_RTU_Funcao_Erro(MODBUS_RTU_Codigo_Erro_Illegal_Data_Address);
	}
	else MODBUS_RTU_Funcao_Erro(MODBUS_RTU_Codigo_Erro_Illegal_Function);
}

void MODBUS_RTU_Funcao_Read_Discrete_Input()			// FUNÇÃO 02
{
	if (Quantidade_Discrete_Inputs > 0)
	{
		MODBUS_RTU_Endereco = (MODBUS_RTU_Buffer_RX[2] << 8) | (MODBUS_RTU_Buffer_RX[3]);
		MODBUS_RTU_Quantidade_Bits = (MODBUS_RTU_Buffer_RX[4] << 8) | (MODBUS_RTU_Buffer_RX[5]);
		MODBUS_RTU_Quantidade_Bytes = (MODBUS_RTU_Quantidade_Bits + 7) / 8;

		if ((MODBUS_RTU_Endereco + MODBUS_RTU_Quantidade_Bits) <= MIN(Quantidade_Discrete_Inputs, 9999))
		{
			MODBUS_RTU_Buffer_TX[0] = MODBUS_RTU_Escravo;												//Endereço de Escravo
			MODBUS_RTU_Buffer_TX[1] = MODBUS_RTU_Codigo_Read_Discrete_Input;							//Código da Função
			MODBUS_RTU_Buffer_TX[2] = MODBUS_RTU_Quantidade_Bytes;										//Número de Bytes de Dados

			uint8_t Contador = 0;
			for (int i = 0; i < MODBUS_RTU_Quantidade_Bits; i++)
			{
			    if (Contador < 8)
			    {
			    	if (Discrete_Inputs[i + MODBUS_RTU_Endereco] == 1)
			    	{
			    		SET_Bit (MODBUS_RTU_Buffer_TX[3 + (i / 8)], Contador);
			    	}
			    	else
			    	{
			    		CLEAR_Bit (MODBUS_RTU_Buffer_TX[3 + (i / 8)], Contador);
			    	}
			        Contador++;
			    }
			    if (Contador >= 8) Contador = 0;
			}

			//Executa Função do Cálculo do CRC
			MODBUS_CRC = Calculo_CRC_MODBUS(MODBUS_RTU_Buffer_TX, MODBUS_RTU_Quantidade_Bytes + 3);
			MODBUS_RTU_Buffer_TX[MODBUS_RTU_Quantidade_Bytes + 3] = MODBUS_CRC & 0xFF;
			MODBUS_RTU_Buffer_TX[MODBUS_RTU_Quantidade_Bytes + 4] = MODBUS_CRC >> 8;

			HAL_UART_Transmit(&MODBUS_RTU_USART, MODBUS_RTU_Buffer_TX, MODBUS_RTU_Quantidade_Bytes + 5, MODBUS_RTU_Timeout);
		}
		else MODBUS_RTU_Funcao_Erro(MODBUS_RTU_Codigo_Erro_Illegal_Data_Address);
	}
	else MODBUS_RTU_Funcao_Erro(MODBUS_RTU_Codigo_Erro_Illegal_Function);
}

void MODBUS_RTU_Funcao_Read_Holding_Register()			// FUNÇÃO 03
{
	if (Quantidade_Holding_Registers > 0)
	{
		MODBUS_RTU_Endereco = (MODBUS_RTU_Buffer_RX[2] << 8) | (MODBUS_RTU_Buffer_RX[3]);
		MODBUS_RTU_Quantidade_Registers = (MODBUS_RTU_Buffer_RX[4] << 8) | (MODBUS_RTU_Buffer_RX[5]);

		if ((MODBUS_RTU_Endereco + MODBUS_RTU_Quantidade_Registers) <= MIN(Quantidade_Holding_Registers, 9999))
		{
			MODBUS_RTU_Quantidade_Bytes = MODBUS_RTU_Quantidade_Registers * 2;

			MODBUS_RTU_Buffer_TX[0] = MODBUS_RTU_Escravo;												//Endereço de Escravo
			MODBUS_RTU_Buffer_TX[1] = MODBUS_RTU_Codigo_Read_Holding_Register;							//Código da Função
			MODBUS_RTU_Buffer_TX[2] = MODBUS_RTU_Quantidade_Bytes;										//Número de Bytes de Dados

			for (int i = 0; i <= (MODBUS_RTU_Quantidade_Registers - 1); i++)
			{
				MODBUS_RTU_Buffer_TX[3 + (i * 2)] = (Holding_Registers[i + MODBUS_RTU_Endereco]) >> 8;
				MODBUS_RTU_Buffer_TX[4 + (i * 2)] = (Holding_Registers[i + MODBUS_RTU_Endereco]) & 0xFF;
			}

			//Executa Função do Cálculo do CRC
			MODBUS_CRC = Calculo_CRC_MODBUS(MODBUS_RTU_Buffer_TX, MODBUS_RTU_Quantidade_Bytes + 3);
			MODBUS_RTU_Buffer_TX[MODBUS_RTU_Quantidade_Bytes + 3] = MODBUS_CRC & 0xFF;
			MODBUS_RTU_Buffer_TX[MODBUS_RTU_Quantidade_Bytes + 4] = MODBUS_CRC >> 8;

			HAL_UART_Transmit(&MODBUS_RTU_USART, MODBUS_RTU_Buffer_TX, MODBUS_RTU_Quantidade_Bytes + 5, MODBUS_RTU_Timeout);
		}
		else MODBUS_RTU_Funcao_Erro(MODBUS_RTU_Codigo_Erro_Illegal_Data_Address);
	}
	else MODBUS_RTU_Funcao_Erro(MODBUS_RTU_Codigo_Erro_Illegal_Function);
}

void MODBUS_RTU_Funcao_Read_Input_Register()			// FUNÇÃO 04
{
	if (Quantidade_Input_Registers > 0)
	{
		MODBUS_RTU_Endereco = (MODBUS_RTU_Buffer_RX[2] << 8) | (MODBUS_RTU_Buffer_RX[3]);
		MODBUS_RTU_Quantidade_Registers = (MODBUS_RTU_Buffer_RX[4] << 8) | (MODBUS_RTU_Buffer_RX[5]);

		if ((MODBUS_RTU_Endereco + MODBUS_RTU_Quantidade_Registers) <= MIN(Quantidade_Input_Registers, 9999))
		{
			MODBUS_RTU_Quantidade_Bytes = MODBUS_RTU_Quantidade_Registers * 2;

			MODBUS_RTU_Buffer_TX[0] = MODBUS_RTU_Escravo;												//Endereço de Escravo
			MODBUS_RTU_Buffer_TX[1] = MODBUS_RTU_Codigo_Read_Input_Register;							//Código da Função
			MODBUS_RTU_Buffer_TX[2] = MODBUS_RTU_Quantidade_Bytes;										//Número de Bytes de Dados

			for (int i = 0; i <= MODBUS_RTU_Quantidade_Registers - 1; i++)
			{
				MODBUS_RTU_Buffer_TX[3 + (i * 2)] = (Input_Registers[i + MODBUS_RTU_Endereco]) >> 8;
				MODBUS_RTU_Buffer_TX[4 + (i * 2)] = (Input_Registers[i + MODBUS_RTU_Endereco]) & 0xFF;
			}

			//Executa Função do Cálculo do CRC
			MODBUS_CRC = Calculo_CRC_MODBUS(MODBUS_RTU_Buffer_TX, MODBUS_RTU_Quantidade_Bytes + 3);
			MODBUS_RTU_Buffer_TX[MODBUS_RTU_Quantidade_Bytes + 3] = MODBUS_CRC & 0xFF;
			MODBUS_RTU_Buffer_TX[MODBUS_RTU_Quantidade_Bytes + 4] = MODBUS_CRC >> 8;

			HAL_UART_Transmit(&MODBUS_RTU_USART, MODBUS_RTU_Buffer_TX, MODBUS_RTU_Quantidade_Bytes + 5, MODBUS_RTU_Timeout);
		}
		else MODBUS_RTU_Funcao_Erro(MODBUS_RTU_Codigo_Erro_Illegal_Data_Address);
	}
	else MODBUS_RTU_Funcao_Erro(MODBUS_RTU_Codigo_Erro_Illegal_Function);
}

void MODBUS_RTU_Funcao_Write_Single_Coil()				// FUNÇÃO 05
{
	if (Quantidade_Coils > 0)
	{
		MODBUS_RTU_Endereco = (MODBUS_RTU_Buffer_RX[2] << 8) | (MODBUS_RTU_Buffer_RX[3]);

		if (MODBUS_RTU_Endereco <= MIN((Quantidade_Coils - 1), 9998))
		{
			if ((MODBUS_RTU_Buffer_RX[4] == 0 || MODBUS_RTU_Buffer_RX[4] == 255) && (MODBUS_RTU_Buffer_RX[5] == 0))
			{
				if (MODBUS_RTU_Buffer_RX[4] == 255) Coils[MODBUS_RTU_Endereco] = 1;
				else Coils[MODBUS_RTU_Endereco] = 0;

				if (MODBUS_RTU_Buffer_RX[0] != 0)
				HAL_UART_Transmit(&MODBUS_RTU_USART, MODBUS_RTU_Buffer_RX, 8, MODBUS_RTU_Timeout);
			}
			else MODBUS_RTU_Funcao_Erro(MODBUS_RTU_Codigo_Erro_Illegal_Data_Value);
		}
		else MODBUS_RTU_Funcao_Erro(MODBUS_RTU_Codigo_Erro_Illegal_Data_Address);
	}
	else MODBUS_RTU_Funcao_Erro(MODBUS_RTU_Codigo_Erro_Illegal_Function);
}

void MODBUS_RTU_Funcao_Write_Single_Register()			// FUNÇÃO 06
{
	if (Quantidade_Holding_Registers > 0)
	{
		MODBUS_RTU_Endereco = (MODBUS_RTU_Buffer_RX[2] << 8) | (MODBUS_RTU_Buffer_RX[3]);

		if (MODBUS_RTU_Endereco <= MIN((Quantidade_Holding_Registers - 1), 9998))
		{
			Holding_Registers[MODBUS_RTU_Endereco] = (MODBUS_RTU_Buffer_RX[4] << 8) | (MODBUS_RTU_Buffer_RX[5]);

			if (MODBUS_RTU_Buffer_RX[0] != 0)
			HAL_UART_Transmit(&MODBUS_RTU_USART, MODBUS_RTU_Buffer_RX, 8, MODBUS_RTU_Timeout);
		}
		else MODBUS_RTU_Funcao_Erro(MODBUS_RTU_Codigo_Erro_Illegal_Data_Address);
	}
	else MODBUS_RTU_Funcao_Erro(MODBUS_RTU_Codigo_Erro_Illegal_Function);
}

void MODBUS_RTU_Funcao_Write_Multiple_Coils()			// FUNÇÃO 15
{
	if (Quantidade_Coils > 0)
	{
		MODBUS_RTU_Endereco = (MODBUS_RTU_Buffer_RX[2] << 8) | (MODBUS_RTU_Buffer_RX[3]);
		MODBUS_RTU_Quantidade_Bits = (MODBUS_RTU_Buffer_RX[4] << 8) | (MODBUS_RTU_Buffer_RX[5]);

		if ((MODBUS_RTU_Endereco + MODBUS_RTU_Quantidade_Bits) <= MIN(Quantidade_Coils, 9999))
		{

			for (int i = 0; i <= MODBUS_RTU_Quantidade_Bits - 1; i++)
			{
			    // Obter o byte certo (i / 8) e o bit certo dentro do byte (i % 8)
			    Coils[i + MODBUS_RTU_Endereco] = (MODBUS_RTU_Buffer_RX[7 + (i / 8)] & (1 << (i % 8))) ? 1 : 0;
			}

			MODBUS_RTU_Buffer_TX[0] = MODBUS_RTU_Escravo;												//Endereço de Escravo
			MODBUS_RTU_Buffer_TX[1] = MODBUS_RTU_Codigo_Write_Multiple_Coils;							//Código da Função
			MODBUS_RTU_Buffer_TX[2] = MODBUS_RTU_Buffer_RX[2];											//
			MODBUS_RTU_Buffer_TX[3] = MODBUS_RTU_Buffer_RX[3];											//
			MODBUS_RTU_Buffer_TX[4] = MODBUS_RTU_Buffer_RX[4];											//
			MODBUS_RTU_Buffer_TX[5] = MODBUS_RTU_Buffer_RX[5];											//

			//Executa Função do Cálculo do CRC
			MODBUS_CRC = Calculo_CRC_MODBUS(MODBUS_RTU_Buffer_TX, 6);
			MODBUS_RTU_Buffer_TX[6] = MODBUS_CRC & 0xFF;
			MODBUS_RTU_Buffer_TX[7] = MODBUS_CRC >> 8;

			if (MODBUS_RTU_Buffer_RX[0] != 0)
			HAL_UART_Transmit(&MODBUS_RTU_USART, MODBUS_RTU_Buffer_TX, 8, MODBUS_RTU_Timeout);
		}
		else MODBUS_RTU_Funcao_Erro(MODBUS_RTU_Codigo_Erro_Illegal_Data_Address);
	}
	else MODBUS_RTU_Funcao_Erro(MODBUS_RTU_Codigo_Erro_Illegal_Function);
}

void MODBUS_RTU_Funcao_Write_Multiple_Registers()		// FUNÇÃO 16
{
	if (Quantidade_Holding_Registers > 0)
	{
		MODBUS_RTU_Endereco = (MODBUS_RTU_Buffer_RX[2] << 8) | (MODBUS_RTU_Buffer_RX[3]);
		MODBUS_RTU_Quantidade_Registers = (MODBUS_RTU_Buffer_RX[4] << 8) | (MODBUS_RTU_Buffer_RX[5]);

		if ((MODBUS_RTU_Endereco + MODBUS_RTU_Quantidade_Registers) <= MIN(Quantidade_Holding_Registers, 9999))
		{
			for (int i = 0; i <= MODBUS_RTU_Quantidade_Registers - 1; i++)
			{
				Holding_Registers[i + MODBUS_RTU_Endereco] = (MODBUS_RTU_Buffer_RX[(i * 2) + 7] << 8) | MODBUS_RTU_Buffer_RX[(i * 2) + 8];
			}

			MODBUS_RTU_Buffer_TX[0] = MODBUS_RTU_Escravo;												//Endereço de Escravo
			MODBUS_RTU_Buffer_TX[1] = MODBUS_RTU_Codigo_Write_Multiple_Registers;						//Código da Função
			MODBUS_RTU_Buffer_TX[2] = MODBUS_RTU_Buffer_RX[2];											//
			MODBUS_RTU_Buffer_TX[3] = MODBUS_RTU_Buffer_RX[3];											//
			MODBUS_RTU_Buffer_TX[4] = MODBUS_RTU_Buffer_RX[4];											//
			MODBUS_RTU_Buffer_TX[5] = MODBUS_RTU_Buffer_RX[5];											//

			//Executa Função do Cálculo do CRC
			MODBUS_CRC = Calculo_CRC_MODBUS(MODBUS_RTU_Buffer_TX, 6);
			MODBUS_RTU_Buffer_TX[6] = MODBUS_CRC & 0xFF;
			MODBUS_RTU_Buffer_TX[7] = MODBUS_CRC >> 8;

			if (MODBUS_RTU_Buffer_RX[0] != 0)
			HAL_UART_Transmit(&MODBUS_RTU_USART, MODBUS_RTU_Buffer_TX, 8, MODBUS_RTU_Timeout);
		}
		else MODBUS_RTU_Funcao_Erro(MODBUS_RTU_Codigo_Erro_Illegal_Data_Address);
	}
	else MODBUS_RTU_Funcao_Erro(MODBUS_RTU_Codigo_Erro_Illegal_Function);
}

//==============================================================================
// FUNÇÕES DE INTERRUPÇÕES
//==============================================================================

//Interrupção de Recebimento Serial
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Tamanho)
{
	__disable_irq();
	memcpy(MODBUS_RTU_Buffer_RX, MODBUS_RTU_Buffer_RX_Auxiliar, Tamanho);
	__enable_irq();

	if (huart == &MODBUS_RTU_USART)	//Verifica se a Usart do MODBUS Recebeu Dados Serial
	{

//		memcpy(MODBUS_RTU_Buffer_RX, MODBUS_RTU_Buffer_RX_Auxiliar, MODBUS_RTU_Tamanho_Buffer_RX);

		if (Tamanho > MODBUS_RTU_Tamanho_Buffer_RX)
		{
		    // Descarte a mensagem, pois o tamanho excedeu o buffer
		    HAL_UARTEx_ReceiveToIdle_DMA(&MODBUS_RTU_USART, MODBUS_RTU_Buffer_RX_Auxiliar, MODBUS_RTU_Tamanho_Buffer_RX);
		    __HAL_DMA_DISABLE_IT(&MODBUS_RTU_HDMA_USART_RX, DMA_IT_HT);
		    return;
		}

		// Verifique e limpe erros da UART
		if (__HAL_UART_GET_FLAG(&MODBUS_RTU_USART, UART_FLAG_ORE))
		{
		    __HAL_UART_CLEAR_OREFLAG(&MODBUS_RTU_USART);
		}
		if (__HAL_UART_GET_FLAG(&MODBUS_RTU_USART, UART_FLAG_FE))
		{
		    __HAL_UART_CLEAR_FEFLAG(&MODBUS_RTU_USART);
		}
		if (__HAL_UART_GET_FLAG(&MODBUS_RTU_USART, UART_FLAG_NE))
		{
		    __HAL_UART_CLEAR_NEFLAG(&MODBUS_RTU_USART);
		}

		//Verifica o Endereço de Escravo do Modbus
		if (MODBUS_RTU_Buffer_RX[0] == MODBUS_RTU_Escravo && MODBUS_RTU_Buffer_RX[0] != 0 && MODBUS_RTU_Buffer_RX[0] <= 247 && Tamanho >= 8)
		{
			switch (MODBUS_RTU_Buffer_RX[1])
			{
				case MODBUS_RTU_Codigo_Read_Coil:					// Função 01
					MODBUS_RTU_Funcao_Read_Coil();
					break;

				case MODBUS_RTU_Codigo_Read_Discrete_Input:			// Função 02
					MODBUS_RTU_Funcao_Read_Discrete_Input();
					break;

				case MODBUS_RTU_Codigo_Read_Holding_Register:		// Função 03
					MODBUS_RTU_Funcao_Read_Holding_Register();
					break;

				case MODBUS_RTU_Codigo_Read_Input_Register:			// Função 04
					MODBUS_RTU_Funcao_Read_Input_Register();
					break;

				case MODBUS_RTU_Codigo_Write_Single_Coil:			// Função 05
					MODBUS_RTU_Funcao_Write_Single_Coil();
					break;

				case MODBUS_RTU_Codigo_Write_Single_Register:		// Função 06
					MODBUS_RTU_Funcao_Write_Single_Register();
					break;

				case MODBUS_RTU_Codigo_Write_Multiple_Coils:		// Função 15
					MODBUS_RTU_Funcao_Write_Multiple_Coils();
					break;

				case MODBUS_RTU_Codigo_Write_Multiple_Registers:	// Função 16
					MODBUS_RTU_Funcao_Write_Multiple_Registers();
					break;

				default:
					MODBUS_RTU_Funcao_Erro(MODBUS_RTU_Codigo_Erro_Illegal_Function);
					break;
			}
		} // Verificação Escravo MODBUS RTU
		else if (MODBUS_RTU_Buffer_RX[0] == 0 && MODBUS_RTU_Broadcast == 1 && Tamanho >= 8)
		{
			switch (MODBUS_RTU_Buffer_RX[1])
			{
				case MODBUS_RTU_Codigo_Write_Single_Coil:			// Função 05
					MODBUS_RTU_Funcao_Write_Single_Coil();
					break;

				case MODBUS_RTU_Codigo_Write_Single_Register:		// Função 06
					MODBUS_RTU_Funcao_Write_Single_Register();
					break;

				case MODBUS_RTU_Codigo_Write_Multiple_Coils:		// Função 15
					MODBUS_RTU_Funcao_Write_Multiple_Coils();
					break;

				case MODBUS_RTU_Codigo_Write_Multiple_Registers:	// Função 16
					MODBUS_RTU_Funcao_Write_Multiple_Registers();
					break;
			}
		}

		memset(MODBUS_RTU_Buffer_RX_Auxiliar, '\0', sizeof(MODBUS_RTU_Buffer_RX_Auxiliar));

        // Reinicia o DMA para o próximo recebimento
        if (HAL_UARTEx_ReceiveToIdle_DMA(&MODBUS_RTU_USART, MODBUS_RTU_Buffer_RX_Auxiliar, MODBUS_RTU_Tamanho_Buffer_RX) != HAL_OK)
        {
            // Trate erros do DMA, se necessário
            Error_Handler();
        }

        __HAL_DMA_DISABLE_IT(&MODBUS_RTU_HDMA_USART_RX, DMA_IT_HT);

		Bit.WD_MODBUS_RTU = 0;

		if (ContadorErroModbus > 0) ContadorErroModbus--;

		ContadorInterrupcaoUsartModbus++;
	} // Verificação USART MODBUS RTU
} // Interrupção UART

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim3)
	{
		flagI2C = 1;

		if (Bit.M_LED_Erro == 1)
		{
			ContadorLedErro++;

			if (ContadorLedErro >= TEMPO_LED_ERRO)
			{
				HAL_GPIO_WritePin(GPIO_LED_Erro, LED_Erro_Pin, 0);
				ContadorLedErro = 0;
				Bit.M_LED_Erro = 0;
			}
		}
		else
		{
			ContadorLedErro = 0;
		}

		TemporizadorInterrupcaoUsartModbus++;
	} //htim3
} //Interrupção Timers
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	memset(MODBUS_RTU_Buffer_RX_Auxiliar, '\0', sizeof(MODBUS_RTU_Buffer_RX_Auxiliar));
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);

  HAL_GPIO_WritePin(GPIOC, LED_BP_Pin, 1);
  HAL_Delay(200);
  HAL_GPIO_WritePin(GPIOC, LED_BP_Pin, 0); //Led Inicia Ligado (Aparentemente o LED interno é invertido na Black Pill).
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if (flagI2C == 1)
	  {
		  ReadData(rxDataValue, sizeof(rxDataValue));
		  Input_Registers[0] = ((uint16_t)rxDataValue[1] << 8) | rxDataValue[0];

		  Discrete_Inputs[0] = GET_Bit(rxDataValue[0], 0);
		  Discrete_Inputs[1] = GET_Bit(rxDataValue[0], 1);
		  Discrete_Inputs[2] = GET_Bit(rxDataValue[0], 2);
		  Discrete_Inputs[3] = GET_Bit(rxDataValue[0], 3);
		  Discrete_Inputs[4] = GET_Bit(rxDataValue[0], 4);
		  Discrete_Inputs[5] = GET_Bit(rxDataValue[0], 5);
		  Discrete_Inputs[6] = GET_Bit(rxDataValue[0], 6);
		  Discrete_Inputs[7] = GET_Bit(rxDataValue[0], 7);
		  Discrete_Inputs[8] = GET_Bit(rxDataValue[1], 0);
		  Discrete_Inputs[9] = GET_Bit(rxDataValue[1], 1);
		  Discrete_Inputs[10] = GET_Bit(rxDataValue[1], 2);
		  Discrete_Inputs[11] = GET_Bit(rxDataValue[1], 3);
		  flagI2C = 0;
	  }

		if (!__HAL_UART_GET_IT_SOURCE(&MODBUS_RTU_USART, UART_IT_RXNE))
		{
			HAL_UARTEx_ReceiveToIdle_DMA(&MODBUS_RTU_USART, MODBUS_RTU_Buffer_RX_Auxiliar, MODBUS_RTU_Tamanho_Buffer_RX);
		}

		if (TemporizadorInterrupcaoUsartModbus >= 8000)
		{
			if (ContadorInterrupcaoUsartModbus < 4)
			{
		        // Reinicia o DMA para o próximo recebimento
		        if (HAL_UARTEx_ReceiveToIdle_DMA(&MODBUS_RTU_USART, MODBUS_RTU_Buffer_RX_Auxiliar, MODBUS_RTU_Tamanho_Buffer_RX) != HAL_OK)
		        {
		            // Trate erros do DMA, se necessário
		            Error_Handler();
		        }

		        __HAL_DMA_DISABLE_IT(&MODBUS_RTU_HDMA_USART_RX, DMA_IT_HT);
			}

			ContadorInterrupcaoUsartModbus = 0;
			TemporizadorInterrupcaoUsartModbus = 0;

		}

		if (Bit.WD_MODBUS_RTU == 0 && ContadorErroModbus < QTD_ERRO_MODBUS)
		{
		  HAL_IWDG_Refresh(&hiwdg);

		  Bit.WD_MODBUS_RTU = 1;
		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 1500-1;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 10000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_BP_GPIO_Port, LED_BP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_Erro_GPIO_Port, LED_Erro_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_BP_Pin */
  GPIO_InitStruct.Pin = LED_BP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_BP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Erro_Pin */
  GPIO_InitStruct.Pin = LED_Erro_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_Erro_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
