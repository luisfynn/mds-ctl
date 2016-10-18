/**
 *	@file   debug.c
 *	@brief	
 *	@author luisfynn
 *	@date   2016/10/14 17:30
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "common.h"
#include "debug.h"

extern UART_HandleTypeDef huart1;
extern osMessageQId uartRxQueueHandle;

osEvent evt;
SHELLDRV_OBJECT_t	user_shellObj;

uint8_t consoleMessage[7] ="\r\nroot>";

SHELL_CMD_t user_cmd[] = {
	{
		(int8_t*)"led", CmdLed, 
		(int8_t*)"led test - led ex) led [pw/a/b/clear] [on/off/blink_on/blink_off]",
		(int8_t*)"=====================================================\n"
		" LED          |        led [pw] [on/off]\n"
		"              |        led [a]  [on/off]\n"
		"              |        led [b]  [on/off]\n"
		"              |        led [pw] [b_on/b_off]\n"
		"              |        led [a]  [b_on/b_off]\n"
		"              |        led [b]  [b_on/b_off]\n"
		"              |        led [clear]\n"
		"====================================================="
	},
	{
		(int8_t*)"alarm", CmdAlarmOut,
		(int8_t*)"alarm test - alarm ex) alarm [open/close]",
		(int8_t*)"\n=====================================================\n"
		" ALARM OUT    |        alarm [open/close]\n"
		"====================================================="
	}
};

static void ShellShowReayBanner(SHELLDRV_OBJECT_t* _sObj)
{
	uint8_t	i;
	int8_t	banner[][MAX_SIZE_MESSAGE] = {
		"--------------------------------------------",
		"|| Start the alarm out board for MDS ^_^* ||",
		"||    please push the your enter key      ||",
		"--------------------------------------------",
	};

	for (i = 0; i < 4; i++) {
		printf("%s\n", banner[i]);
	}
}

static void ShellSetCmdMaxLen(SHELLDRV_OBJECT_t* _sObj)
{
	uint8_t i, len = 0;

	for (i = 0; i < _sObj->cmdListCnt; i++) {
		if (len < (strlen((char*)_sObj->cmdList[i].cmdName))) {
			len = strlen((char*)_sObj->cmdList[i].cmdName);
		}
	}
	_sObj->cmdMaxLen = len;
}

static void ShellInitRecvBuf(SHELL_INBUF_t* _sObjBuf)
{
	memset((char*)_sObjBuf->msgBuf, 0, MAX_SIZE_MESSAGE);
	_sObjBuf->idx = 0;
	//_sObjBuf->isCmdMode = SHELL_FALSE;
}

static void InitShellDrv(SHELLDRV_OBJECT_t* _sObj)
{
	ShellSetCmdMaxLen(_sObj);
	ShellInitRecvBuf(&(_sObj->shellBufData));
	ShellShowReayBanner(_sObj);
}

void InitDebug(void)
{
	user_shellObj.cmdList = user_cmd;
	user_shellObj.cmdListCnt = sizeof(user_cmd)/sizeof(SHELL_CMD_t);
	InitShellDrv(&user_shellObj);
}

static int8_t ShellSaveRecv(SHELL_INBUF_t* _sInBuf, int8_t d)
{
	if (_sInBuf->idx < MAX_SIZE_MESSAGE) {
		_sInBuf->msgBuf[_sInBuf->idx++] = d;
		return SHELL_TRUE;
	}
	return SHELL_FALSE;
}

static void ShellDispError(SHELLDRV_OBJECT_t* _sObj, uint8_t* name)
{
	printf("\n%s is unknown command!\n", name);
}

static void ShellShowCmdHelpUsage(SHELLDRV_OBJECT_t* _sObj, int8_t* str)
{
	if (str == NULL) {
		printf("\nnot applicable (N/A)\n" );
	} else {
		printf("\n%s", str);
	}
}

static void ShellShowTotalCmdHelp(SHELLDRV_OBJECT_t* _sObj)
{
	uint8_t i, size;
	int8_t	makeStr[(_sObj->cmdMaxLen) + 5];		// +5 (space)
	int8_t* makeStr_end;
	SHELL_CMD_t* pos;

	size = sizeof(makeStr);
	pos = _sObj->cmdList;
	makeStr_end = &makeStr[size - 1];
	
	for (i = 0; i < (_sObj->cmdListCnt); i++) {
		*makeStr_end = '\0';	
		memset((char*)makeStr, 0x20, sizeof(makeStr));
		strncpy((char*)makeStr, (char*)pos[i].cmdName, strlen((char*)pos[i].cmdName));

		printf("\n%s\n", makeStr);
		ShellShowCmdHelpUsage(_sObj, pos[i].cmdUsage);
	}
}

static int8_t ShellFindUserCmd(uint8_t* name, SHELLDRV_OBJECT_t* _sObj)
{
	uint8_t cnt;
	SHELL_CMD_t	*pos;

	pos = _sObj->cmdList;				//SHELLDRV_OBJECT_t 의 cmdList 주소를  pos에 넣는다.

	for (cnt = 0; cnt < _sObj->cmdListCnt; cnt++) {
		if (strncmp((char*)name, (char*)pos->cmdName, strlen((char*)name)) != 0) {
			pos++;
			continue;
		}	
		return cnt;
	}
	return SHELL_FALSE;
}

static void ShellExeCommand(SHELLDRV_OBJECT_t* _sObj, SHELL_PARSER_CMD_t* _cmdata)
{
	int8_t idx = SHELL_FALSE;

	idx = ShellFindUserCmd(_cmdata->cmdName, _sObj);

	if (idx == SHELL_FALSE) {
		if (strncmp((char*)_cmdata->cmdName, (char*)"help", strlen((char*)"help")) != 0) {
			ShellDispError(_sObj, _cmdata->cmdName);
		} 
		ShellShowTotalCmdHelp(_sObj);
	} else {
		if (_cmdata->optCount != 0) {
			if (strncmp((char*)_cmdata->optVal[0], (char*)"usage", strlen((char*)"usage")) == 0) {
				ShellShowCmdHelpUsage(_sObj, _sObj->cmdList[idx].cmdUsage); 		
			} else if (strncmp((char*)_cmdata->optVal[0], (char*)"help", strlen((char*)"help")) == 0) {
				ShellShowCmdHelpUsage(_sObj, _sObj->cmdList[idx].cmdHelp); 		
			} else {
				if (_sObj->cmdList[idx].CmdExecFunc == NULL) {
					ShellDispError(_sObj, _cmdata->cmdName);
				} else {
					_sObj->cmdList[idx].CmdExecFunc(_cmdata->optCount, _cmdata->optVal);	
				}
			}
		} else {
			ShellShowCmdHelpUsage(_sObj, _sObj->cmdList[idx].cmdHelp); 		
		}
	}
	ShellInitRecvBuf(&(_sObj->shellBufData));
}

static int8_t ShellParserRecv(SHELLDRV_OBJECT_t* _sObj)
{
	int8_t ret = SHELL_TRUE;
	uint8_t tempMsgBuf[MAX_SIZE_MESSAGE];
	uint8_t i, len;
	uint8_t* pos;
	SHELL_PARSER_CMD_t	_sParData;

	memset((char*)&_sParData, 0, sizeof(SHELL_PARSER_CMD_t));
	memset((char*)&tempMsgBuf, 0, sizeof(tempMsgBuf));
	strncpy((char*)&tempMsgBuf, (char*)_sObj->shellBufData.msgBuf, sizeof(_sObj->shellBufData.msgBuf));

	len = strlen((char*)tempMsgBuf);
	pos = tempMsgBuf;
	_sParData.cmdName = pos;		//tempMsgBuf 의 주소를 cmdName에 넣는다.

	for (i = 0; i < len; i++) {
		if (tempMsgBuf[i] == SHELL_SPACE) {
			tempMsgBuf[i] = '\0';
			pos = &(tempMsgBuf[i + 1]);

			if (_sParData.optCount >= MAX_COUNT_CMD_PARAM) {
				ShellDispError(_sObj, _sParData.cmdName);
				ret = SHELL_FALSE;	
				break;
			} else {
				_sParData.optVal[_sParData.optCount++] = pos;		//단어가 시작되는 첫위치를 배열에 저장한다.
			}
		}	
	}

	if (_sParData.cmdName == NULL) {
		ret = SHELL_FALSE;	
	} else {
		if (ret == SHELL_TRUE) {
			ShellExeCommand(_sObj, &_sParData);
		}
	}

	ShellInitRecvBuf(&(_sObj->shellBufData));
	return ret;
}

static void PutDebugMessage(int8_t type, int8_t* str)
{
	char name[16], message[64];
	memset(name, 0, sizeof(name));
	memset(message, 0, sizeof(message));

	if (type == ALARM_OUT) {
		strncpy((char*)name, (char*)"Alarm Out", strlen((char*)"Alarm Out"));		
	} else if (type == DVR1_LED) {
		strncpy((char*)name, (char*)"SetB", strlen((char*)"SetB"));		
	} else if (type == DVR2_LED) {
		strncpy((char*)name, (char*)"SetA", strlen((char*)"SetA"));		
	} else if (type == LIVE_LED) {
		strncpy((char*)name, (char*)"Power", strlen((char*)"Power"));		
	} else {
		strncpy((char*)name, (char*)"Clear", strlen((char*)"Clear"));		
	}

	sprintf(message, "%s: %s", name, str);
	printf("\n%s", message);
}

void CmdLed(uint8_t _argc, uint8_t* _argv[])
{
	int8_t sellSt =  SHELL_FALSE;
	int8_t ledSt = PORT_MAX_CNT;

	if ((strncmp((char*)_argv[0], (char*)"B", strlen((char*)"B")) == 0) 
		|| (strncmp((char*)_argv[0], (char*)"b", strlen((char*)"b")) == 0)) {
		ledSt = DVR1_LED;
	} else if ((strncmp((char*)_argv[0], (char*)"A", strlen((char*)"A")) == 0) 
		|| (strncmp((char*)_argv[0], (char*)"a", strlen((char*)"a")) == 0)) {
		ledSt = DVR2_LED;
	} else if ((strncmp((char*)_argv[0], (char*)"pw", strlen((char*)"pw")) == 0) 
		|| (strncmp((char*)_argv[0], (char*)"power", strlen((char*)"power")) == 0)) {
		ledSt = LIVE_LED;
	} 

	if (ledSt == PORT_MAX_CNT) {
		if (strncmp((char*)_argv[0], (char*)"clear", strlen((char*)"clear")) == 0) {
			//GpioOutDriveSetup(LIVE_LED, OFF);
			//GpioOutDriveSetup(DVR1_LED, OFF);
			//GpioOutDriveSetup(DVR2_LED, OFF);

			//GpioToggleSetup(LIVE_LED, OFF);
			//GpioToggleSetup(DVR1_LED, OFF);
			//GpioToggleSetup(DVR2_LED, OFF);
			sellSt = SHELL_TRUE;	

			PutDebugMessage(ledSt, (int8_t*)"All Off");
		}
	} else {
		if (strncmp((char*)_argv[1],(char*)"on", strlen((char*)"on")) == 0) {
			//GpioToggleSetup(ledSt, OFF);
			//GpioOutDriveSetup(ledSt, ON);
			sellSt = SHELL_TRUE;	

			PutDebugMessage(ledSt, (int8_t*)"Led On");
		} else if (strncmp((char*)_argv[1], (char*)"off", strlen((char*)"off")) == 0) {
			//GpioToggleSetup(ledSt, OFF);
			//GpioOutDriveSetup(ledSt, OFF);
			sellSt = SHELL_TRUE;	

			PutDebugMessage(ledSt, (int8_t*)"Led Off");
		} else if ((strncmp((char*)_argv[1], (char*)"blink_on", strlen((char*)"blink_on")) == 0)
				|| (strncmp((char*)_argv[1], (char*)"b_on", strlen((char*)"b_on")) == 0) ) {
			//GpioToggleSetup(ledSt, ON);
			sellSt = SHELL_TRUE;	

			PutDebugMessage(ledSt, (int8_t*)"Led Blink On");
		} else if ((strncmp((char*)_argv[1], (char*)"blink_off", strlen((char*)"blink_off")) == 0)
				|| (strncmp((char*)_argv[1], (char*)"b_off", strlen((char*)"b_off")) == 0) ) {
			//GpioToggleSetup(ledSt, OFF);
			sellSt = SHELL_TRUE;	

			PutDebugMessage(ledSt, (int8_t*)"Led Blink Off");
		} 
	}

	if (sellSt == SHELL_FALSE) {
		ShellShowCmdHelpUsage(&user_shellObj, user_cmd[CMD_ALARM].cmdHelp);
	}
}

void CmdAlarmOut(uint8_t _argc, uint8_t* _argv[])
{
	int8_t sellSt =  SHELL_FALSE;

	if (strncmp((char*)_argv[0], (char*)"close", strlen((char*)"close")) == 0) {
		//GpioOutDriveSetup(ALARM_OUT, OFF);
		sellSt = SHELL_TRUE;
		PutDebugMessage(ALARM_OUT, (int8_t*)"N.C");
	} else if (strncmp((char*)_argv[0], (char*)"open", strlen((char*)"open")) == 0) {
		//GpioOutDriveSetup(ALARM_OUT, ON);
		sellSt = SHELL_TRUE;
		PutDebugMessage(ALARM_OUT, (int8_t*)"N.O");
	}
	if (sellSt == SHELL_FALSE) {
		ShellShowCmdHelpUsage(&user_shellObj, user_cmd[CMD_ALARM].cmdHelp);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	int8_t	ret;
	SHELL_INBUF_t* _sInBuf;
	SHELLDRV_OBJECT_t* _sObj = &user_shellObj;

	_sInBuf = &(_sObj->shellBufData);

  if(huart->Instance == USART1)
  {
	  evt = osMessageGet(uartRxQueueHandle, 0);

	  if(evt.status == osEventMessage)
	  {
		  uint8_t recvData = evt.value.v & BIT8_MASK;

		  if(recvData == SHELL_CR)
		  {
			  if(_sInBuf->idx != 0){
				  ShellParserRecv(_sObj);
				  _sInBuf->idx = 0;
			  }else{
				  _sInBuf->idx = 0;
			  }
			  HAL_UART_Transmit_IT(&huart1, consoleMessage, sizeof(consoleMessage));
		  }
		  else if(recvData == SHELL_BACKSPACE || recvData == SHELL_DELETE)
		  {
			  if(_sInBuf->idx > 0)
			  {
				  printf("\b \b");
				  _sInBuf->idx--;
			  }
			  else
			  {
				  _sInBuf->idx = 0;
			  }
		  }
		  else
		  {
			  HAL_UART_Transmit_IT(&huart1, &recvData, USART_TX_BUFF_SIZE);

			  ret = ShellSaveRecv(_sInBuf, recvData);
			  if (ret == SHELL_FALSE) {
						ShellInitRecvBuf(_sInBuf);
			  }
		  }
	  }
	  HAL_UART_Receive_IT(&huart1,uartRxQueueHandle, USART_RX_BUFF_SIZE);
  }
}
