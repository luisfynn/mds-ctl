/*
 * command.h
 *
 *  Created on: 2016. 10. 24.
 *      Author: luisfynn
 */

#ifndef COMMAND_H_
#define COMMAND_H_

#include "shell.h"

/* Default to a width of 8 characters for help message command width */
#ifndef CONFIG_SYS_HELP_CMD_WIDTH
#define CONFIG_SYS_HELP_CMD_WIDTH	8
#endif

#ifndef	__ASSEMBLY__
/*
 * Monitor Command Table
 */

typedef struct cmd_tbl_s {
	char	*name;		/* Command Name			*/
	int		maxargs;	/* maximum number of arguments	*/
	int		repeatable;	/* autorepeat allowed?		*/
	/* Implementation function	*/
	int		(*cmd)(struct cmd_tbl_s *, int, int, char *[]);
	char	*usage;		/* Usage message	(short)	*/
#ifdef	CONFIG_SYS_LONGHELP
	char	*help;		/* Help  message	(long)	*/
#endif
#ifdef CONFIG_AUTO_COMPLETE
	/* do auto completion on the arguments */
	int		(*complete)(int argc, char *argv[], char last_char, int maxv, char *cmdv[]);
#endif
}cmd_tbl_t;

cmd_tbl_t  __shell_cmd_start;
cmd_tbl_t  __shell_cmd_end;

/* common/command.c */
int _do_help (cmd_tbl_t *cmd_start, int cmd_items, cmd_tbl_t * cmdtp, int
	      flag, int argc, char *argv[]);
cmd_tbl_t *find_cmd(const char *cmd);
cmd_tbl_t *find_cmd_tbl (const char *cmd, cmd_tbl_t *table, int table_len);

extern int cmd_usage(cmd_tbl_t *cmdtp);

#ifdef CONFIG_AUTO_COMPLETE
extern void install_auto_complete(void);
extern int cmd_auto_complete(const char *const prompt, char *buf, int *np, int *colp);
#endif

/*
 * Monitor Command
 *
 * All commands use a common argument format:
 *
 * void function (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);
 */

typedef	void	command_t (cmd_tbl_t *, int, int, char *[]);

#if defined(CONFIG_CMD_MEMORY)		\
    || defined(CONFIG_CMD_I2C)		\
    || defined(CONFIG_CMD_ITEST)	\
    || defined(CONFIG_CMD_PCI)		\
    || defined(CONFIG_CMD_PORTIO)
#define CMD_DATA_SIZE
extern int cmd_get_data_size(char* arg, int default_size);
#endif

#endif	/* __ASSEMBLY__ */

/*
 * Command Flags:
 */
#define CMD_FLAG_REPEAT		0x0001	/* repeat last command		*/
#define CMD_FLAG_BOOTD		0x0002	/* command is from bootd	*/

#define Struct_Section  __attribute__ ((section (".shell_cmd")))

#ifdef  CONFIG_SYS_LONGHELP

#define SHELL_CMD(name,maxargs,rep,cmd,usage,help) \
Struct_Section cmd_tbl_t __shell_cmd_##name = {#name, maxargs, rep, cmd, usage, help}

#define SHELL_CMD_MKENT(name,maxargs,rep,cmd,usage,help) \
{#name, maxargs, rep, cmd, usage, help}

#else	/* no long help info */

#define SHELL_CMD(name,maxargs,rep,cmd,usage,help) \
cmd_tbl_t __shell_cmd_##name Struct_Section = {#name, maxargs, rep, cmd, usage}

#define SHELL_CMD_MKENT(name,maxargs,rep,cmd,usage,help) \
{#name, maxargs, rep, cmd, usage}

#endif	/* CONFIG_SYS_LONGHELP */

unsigned long simple_strtoul(const char *cp,char **endp,unsigned int base);
long simple_strtol(const char *cp,char **endp,unsigned int base);


#endif /* COMMAND_H_ */
