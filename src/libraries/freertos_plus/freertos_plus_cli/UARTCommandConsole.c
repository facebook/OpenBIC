/*
 * FreeRTOS Kernel V10.3.0
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/*
 * NOTE:  This file uses a third party USB CDC driver.
 */

/* Standard includes. */
#include "string.h"
#include "stdio.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Example includes. */
#include "config.h"
#include "FreeRTOS_CLI.h"

/* Demo application includes. */
#include "serial.h"

/*-------------------------------------------------------------*
 *		Macros & definitions				*
 *-------------------------------------------------------------*/
#define SHELL_ASCII_NUL				0x00
#define SHELL_ASCII_BEL				0x07
#define SHELL_ASCII_BS				0x08
#define SHELL_ASCII_HT				0x09
#define SHELL_ASCII_LF				0x0A
#define SHELL_ASCII_CR				0x0D
#define SHELL_ASCII_ESC				0x1B
#define SHELL_ASCII_DEL				0x7F
#define SHELL_ASCII_US				0x1F
#define SHELL_ASCII_SP				0x20
#define SHELL_VT100_ARROWUP			'A'
#define SHELL_VT100_ARROWDOWN		'B'
#define SHELL_VT100_ARROWRIGHT		'C'
#define SHELL_VT100_ARROWLEFT		'D'


/* Dimensions the buffer into which history shell command are placed. */
#ifndef CONFIG_CMD_HISTORY_SIZE
#define CONFIG_CMD_HISTORY_SIZE		10
#endif

/* Dimensions the buffer into which input characters are placed. */
#ifndef CONFIG_CMD_MAX_INPUT_SIZE
#define CONFIG_CMD_MAX_INPUT_SIZE		50
#endif

/* Dimentions a buffer to be used by the UART driver, if the UART driver uses a
buffer at all. */
#define cmdQUEUE_LENGTH			25

/* DEL acts as a backspace. */
#define cmdASCII_DEL		( 0x7F )

/* The maximum time to wait for the mutex that guards the UART to become
available. */
#define cmdMAX_MUTEX_WAIT		pdMS_TO_TICKS( 300 )

#ifndef configCLI_BAUD_RATE
	#define configCLI_BAUD_RATE	115200
#endif

/**
 * This is the main buffer to store received characters from the user´s terminal
 */
char cInputString[CONFIG_CMD_MAX_INPUT_SIZE];

/**
 * This is the history buffer to store received command from the user´s terminal
 */
char cInputHistory[CONFIG_CMD_HISTORY_SIZE][CONFIG_CMD_MAX_INPUT_SIZE];

/*-----------------------------------------------------------*/

/*
 * The task that implements the command console processing.
 */
static void prvUARTCommandConsoleTask( void *pvParameters );
void vUARTCommandConsoleStart( uint16_t usStackSize, UBaseType_t uxPriority );

/*-----------------------------------------------------------*/

/* Const messages output by the command console. */
static const char * const pcWelcomeMessage = "FreeRTOS command server.\r\nType Help to view a list of registered commands.\r\n";
static const char * const pcPromptMessage = "minibmc>";
//static const char * const pcNewLine = "\r\n";
static const char * const pcNewLine = "\n";
static const char *const pcClearLine = "\033[2K";

/* Used to guard access to the UART in case messages are sent to the UART from
more than one task. */
static SemaphoreHandle_t xTxMutex = NULL;

/* The handle to the UART port, which is not used by all ports. */
static xComPortHandle xPort = 0;

/*-----------------------------------------------------------*/

static inline void vOutputString( const char * const pcMessage )
{
	vSerialPutString( xPort, ( signed char * ) pcMessage, ( unsigned short ) strlen( pcMessage ) );
}
/*-----------------------------------------------------------*/

static inline void vOutputChar( signed char cOutChar )
{
	xSerialPutChar( xPort, cOutChar, portMAX_DELAY);
}
/*-----------------------------------------------------------*/

void vUARTCommandConsoleStart( uint16_t usStackSize, UBaseType_t uxPriority )
{
	/* Create the semaphore used to access the UART Tx. */
	xTxMutex = xSemaphoreCreateMutex();
	configASSERT( xTxMutex );

	/* Create that task that handles the console itself. */
	xTaskCreate( 	prvUARTCommandConsoleTask,	/* The task that implements the command console. */
					"CLI",						/* Text name assigned to the task.  This is just to assist debugging.  The kernel does not use this name itself. */
					usStackSize,				/* The size of the stack allocated to the task. */
					NULL,						/* The parameter is not used, so NULL is passed. */
					uxPriority,					/* The priority allocated to the task. */
					NULL );						/* A handle is not required, so just pass NULL. */
}
/*-----------------------------------------------------------*/

static void prvUARTCommandConsoleTask( void *pvParameters )
{
	signed char cRxedChar;
	uint8_t cursorIndex = 0;
	uint8_t ucInputIndex = 0;
	uint8_t hist_length = 0;
	uint8_t index_hist = 0;
	uint8_t index_hist_find = 0;
	uint8_t vt100 = 0;
	char *pcOutputString;
	BaseType_t xReturned;

	( void ) pvParameters;

	/* Clear the cmd history buffer */
	for (index_hist = 0; index_hist < CONFIG_CMD_HISTORY_SIZE; index_hist++){
        memset(cInputHistory[index_hist], 0, CONFIG_CMD_MAX_INPUT_SIZE);
    }
	index_hist = 0;
	/* Obtain the address of the output buffer.  Note there is no mutual
	exclusion on this buffer as it is assumed only one command console interface
	will be used at any one time. */
	pcOutputString = FreeRTOS_CLIGetOutputBuffer();

	/* Initialise the UART. */
	xPort = xSerialPortInitMinimal( configCLI_BAUD_RATE, cmdQUEUE_LENGTH );

	/* Send the welcome message. */
	if( xSemaphoreTake( xTxMutex, cmdMAX_MUTEX_WAIT ) == pdPASS )
	{
		vOutputString(pcWelcomeMessage);
		vOutputString(pcPromptMessage);
		/* Must ensure to give the mutex back. */
		xSemaphoreGive( xTxMutex );
	}

	for( ;; )
	{
		uint8_t cr = 0;
		/* Wait for the next character.  The while loop is used in case
		INCLUDE_vTaskSuspend is not set to 1 - in which case portMAX_DELAY will
		be a genuine block time rather than an infinite block time. */
		while( xSerialGetChar( xPort, &cRxedChar, portMAX_DELAY ) != pdPASS );

		/* Ensure exclusive access to the UART Tx. */
		if( xSemaphoreTake( xTxMutex, cmdMAX_MUTEX_WAIT ) == pdPASS )
		{
			if (vt100 == 1){
				switch (cRxedChar) 
				{
					case SHELL_VT100_ARROWUP:
						if (index_hist_find != 0)
							index_hist_find--;
						else
							index_hist_find = hist_length;
						memset(cInputString, 0, CONFIG_CMD_MAX_INPUT_SIZE);
						memcpy(cInputString,
						cInputHistory[index_hist_find],
						strlen(cInputHistory
								[index_hist_find]));
						vOutputChar('\r');
						vOutputString(pcClearLine);
						vOutputString(pcPromptMessage);
						vOutputString(cInputString);
						ucInputIndex = strlen(cInputString);
						cursorIndex = ucInputIndex;
						vt100 = 0;
						break;

					case SHELL_VT100_ARROWDOWN:
						index_hist_find++;
						if (index_hist_find > hist_length)
							index_hist_find = 0;
						memset(cInputString, 0, CONFIG_CMD_MAX_INPUT_SIZE);
						memcpy(cInputString,
						cInputHistory[index_hist_find],
						strlen(cInputHistory[index_hist_find]));
						vOutputChar('\r');
						vOutputString(pcClearLine);
						vOutputString(pcPromptMessage);
						vOutputString(cInputString);
						ucInputIndex = strlen(cInputString);
						cursorIndex = ucInputIndex;
						vt100 = 0;
						break;
					case SHELL_VT100_ARROWRIGHT:
					case SHELL_VT100_ARROWLEFT:
						vt100 = 0;
						break;
				}
			} else {
				/* Echo the character back. */
				switch (cRxedChar) 
				{
					case SHELL_ASCII_ESC: // For VT100 escape sequences
					case '[':
						vt100 = 1;
						// Process escape sequences: maybe later
						break;

					case SHELL_ASCII_DEL:
						vOutputChar(SHELL_ASCII_BEL);
						break;

					case SHELL_ASCII_HT:
						vOutputChar(SHELL_ASCII_BEL);
						break;

					case SHELL_ASCII_CR: // Enter key pressed
						cInputString[ucInputIndex] = '\0';
						vOutputString(pcNewLine);
						cr = 1;
						break;

					case SHELL_ASCII_BS: // Backspace pressed
						if (cursorIndex > 0)
						{
							cursorIndex--;
							cInputString[cursorIndex] = 0;
							ucInputIndex = strlen(cInputString);
							vOutputChar(SHELL_ASCII_BS);
							vOutputChar(SHELL_ASCII_SP);
							vOutputChar(SHELL_ASCII_BS);
						}
						else
							vOutputChar(SHELL_ASCII_BEL);
						break;
					default:
						// Process printable characters, but ignore other ASCII chars
						if (ucInputIndex < (CONFIG_CMD_MAX_INPUT_SIZE - 1) && cRxedChar >= 0x20 && cRxedChar < 0x7F) 
						{
							cInputString[cursorIndex] = cRxedChar;
							vOutputChar(cRxedChar);
							ucInputIndex = strlen(cInputString);
							cursorIndex++;
						}
				}
			}
			/* Was it the end of the line? */
			if(cr)
			{
				/* See if the command is empty, do nothing */
				if( ucInputIndex != 0 )
				{
					/* Pass the received command to the command interpreter.  The
					command interpreter is called repeatedly until it returns
					pdFALSE	(indicating there is no more output) as it might
					generate more than one string. */
					memset(cInputHistory[index_hist], 0, CONFIG_CMD_MAX_INPUT_SIZE);
					memcpy(cInputHistory[index_hist], cInputString, strlen(cInputString));
					hist_length++;
					if (hist_length >= CONFIG_CMD_HISTORY_SIZE)
						hist_length = CONFIG_CMD_HISTORY_SIZE - 1;
					index_hist++;
				    index_hist = index_hist % CONFIG_CMD_HISTORY_SIZE;
					do
					{
						/* Get the next output string from the command interpreter. */
						xReturned = FreeRTOS_CLIProcessCommand( cInputString, pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE );

						/* Write the generated string to the UART. */
						vOutputString(pcOutputString);

					} while( xReturned != pdFALSE );
				}
				index_hist_find = index_hist;
				vOutputString(pcPromptMessage);
				ucInputIndex = 0;
				cursorIndex = 0;
				memset(cInputString, 0, CONFIG_CMD_MAX_INPUT_SIZE);
			}
			/* Must ensure to give the mutex back. */
			xSemaphoreGive( xTxMutex );
		}
	}
}
/*-----------------------------------------------------------*/
