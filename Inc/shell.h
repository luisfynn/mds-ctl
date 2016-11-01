/*
 * shell.h
 *
 *  Created on: 2016. 10. 24.
 *      Author: luisfynn
 */

#ifndef SHELL_H_
#define SHELL_H_

#define CONFIG_SYS_CBSIZE       64
#define CONFIG_SYS_PROMPT       "AUTOIT>"
#define CONFIG_SYS_MAXARGS      8
#define CONFIG_CMDLINE_EDITING
#define CONFIG_SYS_LONGHELP
#define	MAX_SIZE_MESSAGE	128
#define	SHELL_TRUE			1
#define	SHELL_FALSE			-1
//#define DEBUG_PARSER
//#define CONFIG_AUTO_COMPLETE

typedef struct SHELL_INBUF {
	uint8_t	msgBuf[MAX_SIZE_MESSAGE];
	uint8_t	idx, odx;
} SHELL_INBUF_t;

extern SHELL_INBUF_t* sInBuf;

void stm32ShellCommand(void);
SHELL_INBUF_t* ShellInitRecvBuf(void);
void ShellSaveRecv(SHELL_INBUF_t* _sInBuf, uint8_t d);

uint8_t uart_isrx(void);
void usartPutc(uint8_t ch);
void usartPuts(const char* ch);
uint8_t usartGetC(void);

int ctrlc(void);


#endif /* SHELL_H_ */
