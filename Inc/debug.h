/**
 *	@file   debug.h
 *	@brief	
 *	@author luisfynn <ksw@auto-it.co.kr>
 *	@date   2016/10/14 17:34
 */

#ifndef _DEBUG_HEADER_
#define _DEBUG_HEADER_

/* system include */

/* local include */

/* external variable & function */

#define	ANSI_COLOR_RED		"\x1b[31m"
#define	ANSI_COLOR_GREEN	"\x1b[32m"
#define	ANSI_COLOR_YELLOW	"\x1b[33m"
#define	ANSI_COLOR_BLUE		"\x1b[34m"
#define	ANSI_COLOR_MAGENTA	"\x1b[35m"
#define	ANSI_COLOR_CYAN		"\x1b[36m"
#define	ANSI_COLOR_RESET	"\x1b[37m"

#define	MAX_SIZE_MESSAGE	128
#define	MAX_COUNT_CMD_PARAM	16

#define	SHELL_TRUE			1
#define	SHELL_FALSE			-1

#define	SHELL_NULL			0
#define	SHELL_CR			0x0D
#define	SHELL_LF			0x0A
#define	SHELL_SPACE			0x20
#define	SHELL_BACKSPACE		0x08
#define SHELL_DELETE		0x1B

enum {
	ALARM_OUT = 0,
	DVR1_LED,
	DVR2_LED,
	LIVE_LED,
	PORT_MAX_CNT,
};

enum {
	CMD_LED = 0,
	CMD_ALARM,
};

typedef struct SHELL_INBUF {
	int8_t	msgBuf[MAX_SIZE_MESSAGE];
	uint8_t	idx;
} SHELL_INBUF_t;

typedef struct SHELL_CMD {
	int8_t*		cmdName;
	void		(*CmdExecFunc)(uint8_t _argc, uint8_t* argv[]);
	int8_t*		cmdUsage;
	int8_t*		cmdHelp;
} SHELL_CMD_t;

typedef struct SHELL_PARSER_CMD {
	uint8_t		optCount;
	uint8_t*	cmdName;
	uint8_t*	optVal[MAX_COUNT_CMD_PARAM];
} SHELL_PARSER_CMD_t;

typedef struct SHELLDRV_OBJECT {
	uint8_t			cmdListCnt;
	SHELL_CMD_t*	cmdList;
	uint8_t			cmdMaxLen;
	SHELL_INBUF_t	shellBufData;
} SHELLDRV_OBJECT_t;

void InitDebug(void);
void DebugTask();

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void CmdLed(uint8_t _argc, uint8_t* argv[]);
void CmdAlarmOut(uint8_t _argc, uint8_t* argv[]);

#endif /* _DEBUG_HEADER_*/


