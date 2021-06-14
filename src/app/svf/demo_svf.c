/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include "stdlib.h"
#include "cmsis_os.h"
#include "board_device.h"
#include "objects.h"
#include "uart_aspeed.h"
#include "jtag_aspeed.h"
#include "getopt.h"
#include "usb_api.h"
#include "cache_aspeed.h"
#include "FreeRTOS_CLI.h"
#include "log.h"

extern usb_t usb[];

/* Porting Macro */
#define ERROR_OK                        (0)
#define ERROR_NO_CONFIG_FILE            (-2)
#define ERROR_BUF_TOO_SMALL             (-3)
#define ERROR_FAIL                      (-4)
#define ERROR_WAIT                      (-5)
#define ERROR_TIMEOUT_REACHED           (-6)
#define LOG_ERROR(x...) log_error(x)
#define LOG_INFO(x...) log_info(x)
#define LOG_DEBUG(x...) log_debug(x)
#define log_printf_lf(dbg_lvl, file, line, function, fmt, ...) \
        log_log(dbg_lvl, file, line, fmt,  __VA_ARGS__)

#define XXR_TDI				(1 << 0)
#define XXR_TDO				(1 << 1)
#define XXR_MASK			(1 << 2)
#define XXR_SMASK			(1 << 3)

struct svf_xxr_para {
	int len;
	int data_mask;
	uint8_t *tdi;
	uint8_t *tdo;
	uint8_t *mask;
	uint8_t *smask;
};
enum trst_mode {
	TRST_ON,
	TRST_OFF,
	TRST_Z,
	TRST_ABSENT
};
enum svf_command {
	ENDDR,
	ENDIR,
	FREQUENCY,
	HDR,
	HIR,
	PIO,
	PIOMAP,
	RUNTEST,
	SDR,
	SIR,
	STATE,
	TDR,
	TIR,
	TRST,
};
struct svf_para {
	int frequency;
	tap_state_t ir_end_state;
	tap_state_t dr_end_state;
	tap_state_t runtest_run_state;
	tap_state_t runtest_end_state;
	enum trst_mode trst_mode;

	struct svf_xxr_para hir_para;
	struct svf_xxr_para hdr_para;
	struct svf_xxr_para tir_para;
	struct svf_xxr_para tdr_para;
	struct svf_xxr_para sir_para;
	struct svf_xxr_para sdr_para;
};
struct svf_check_tdo_para {
	int line_num;		/* used to record line number of the check operation */
	/* so more information could be printed */
	int enabled;		/* check is enabled or not */
	int buffer_offset;	/* buffer_offset to buffers */
	int bit_len;		/* bit length to check */
};

extern jtag_t jtag[2];
extern struct serial_s stdio_uart;
static jtag_t *svf_jtag;
static struct svf_para svf_para;
static const struct svf_para svf_para_init = {
/*	frequency, ir_end_state, dr_end_state, runtest_run_state, runtest_end_state, trst_mode */
	0,			TAP_IDLE,		TAP_IDLE,	TAP_IDLE,		TAP_IDLE,		TRST_Z,
/*	hir_para */
/*	{len,	data_mask,	tdi,	tdo,	mask,	smask}, */
	{0,			0,		NULL,	NULL,	NULL,	NULL},
/*	hdr_para */
/*	{len,	data_mask,	tdi,	tdo,	mask,	smask}, */
	{0,			0,		NULL,	NULL,	NULL,	NULL},
/*	tir_para */
/*	{len,	data_mask,	tdi,	tdo,	mask,	smask}, */
	{0,			0,		NULL,	NULL,	NULL,	NULL},
/*	tdr_para */
/*	{len,	data_mask,	tdi,	tdo,	mask,	smask}, */
	{0,			0,		NULL,	NULL,	NULL,	NULL},
/*	sir_para */
/*	{len,	data_mask,	tdi,	tdo,	mask,	smask}, */
	{0,			0,		NULL,	NULL,	NULL,	NULL},
/*	sdr_para */
/*	{len,	data_mask,	tdi,	tdo,	mask,	smask}, */
	{0,			0,		NULL,	NULL,	NULL,	NULL},
};
static int svf_line_number;
static const char *svf_command_name[14] = {
	"ENDDR",
	"ENDIR",
	"FREQUENCY",
	"HDR",
	"HIR",
	"PIO",
	"PIOMAP",
	"RUNTEST",
	"SDR",
	"SIR",
	"STATE",
	"TDR",
	"TIR",
	"TRST"
};


#define SVF_CHECK_TDO_PARA_SIZE 1024
static struct svf_check_tdo_para *svf_check_tdo_para;
static int svf_check_tdo_para_index;

#define SVF_MAX_BUFFER_SIZE_TO_COMMIT   (1024)
static uint8_t *svf_tdi_buffer, *svf_tdo_buffer, *svf_mask_buffer;
static int svf_buffer_index = 0;

#define isdigit(c) (c >= '0' && c <= '9')

static int svf_atoi_s2ms(const char * s)
{
	int a = 0;
	/* s to ms */
	int e = 3;
	int c;
	while ((c = *s++) != '\0' && isdigit(c)) {
		a = a*10 + (c - '0');
	}
	if (c == '.') {
		while ((c = *s++) != '\0' && isdigit(c)) {
		a = a*10 + (c - '0');
		e = e-1;
		}
	}

	if (c == 'e' || c == 'E') {
		int sign = 1;
		int i = 0;
		c = *s++;
		if (c == '+')
		c = *s++;
		else if (c == '-') {
		c = *s++;
		sign = -1;
		}
		while (isdigit(c)) {
		i = i*10 + (c - '0');
		c = *s++;
		}
		e += i*sign;
	}
	while (e > 0) {
		a *= 10;
		e--;
	}
	while (e < 0) {
		a /= 10;
		e++;
	}
	return a;
}

static int svf_adjust_array_length(uint8_t **arr, int orig_bit_len, int new_bit_len)
{
	int new_byte_len = (new_bit_len + 7) >> 3;

	if ((NULL == *arr) || (((orig_bit_len + 7) >> 3) < ((new_bit_len + 7) >> 3))) {
		if (*arr != NULL) {
			free(*arr);
			*arr = NULL;
		}
		*arr = malloc(new_byte_len);
		if (NULL == *arr && new_byte_len != 0) {
			LOG_ERROR("not enough memory\n");
			return ERROR_FAIL;
		}
		memset(*arr, 0, new_byte_len);
	}
	return ERROR_OK;
}

static int svf_copy_hexstring_to_binary(char *str, uint8_t **bin, int orig_bit_len, int bit_len)
{
	int i, str_len = strlen(str), str_hbyte_len = (bit_len + 3) >> 2;
	uint8_t ch = 0;

	if (ERROR_OK != svf_adjust_array_length(bin, orig_bit_len, bit_len)) {
		LOG_ERROR("fail to adjust length of array\n");
		return ERROR_FAIL;
	}

	/* fill from LSB (end of str) to MSB (beginning of str) */
	for (i = 0; i < str_hbyte_len; i++) {
		ch = 0;
		while (str_len > 0) {
			ch = str[--str_len];

			/* Skip whitespace.  The SVF specification (rev E) is
			 * deficient in terms of basic lexical issues like
			 * where whitespace is allowed.  Long bitstrings may
			 * require line ends for correctness, since there is
			 * a hard limit on line length.
			 */
			if (!isspace(ch)) {
				if ((ch >= '0') && (ch <= '9')) {
					ch = ch - '0';
					break;
				} else if ((ch >= 'A') && (ch <= 'F')) {
					ch = ch - 'A' + 10;
					break;
				} else {
					log_error("invalid hex string\n");
					return -1;
				}
			}

			ch = 0;
		}

		/* write bin */
		if (i % 2) {
			/* MSB */
			(*bin)[i / 2] |= ch << 4;
		} else {
			/* LSB */
			(*bin)[i / 2] = 0;
			(*bin)[i / 2] |= ch;
		}
	}

	/* consume optional leading '0' MSBs or whitespace */
	while (str_len > 0 && ((str[str_len - 1] == '0')
			|| isspace((int) str[str_len - 1])))
		str_len--;

	/* check validity: we must have consumed everything */
	if (str_len > 0 || (ch & ~((2 << ((bit_len - 1) % 4)) - 1)) != 0) {
		log_error("value execeeds length\n");
		return -1;
	}

	return 0;
}

static int svf_parse_cmd_string(char *str, int len, char **argus, int *num_of_argu)
{
	int pos = 0, num = 0, space_found = 1, in_bracket = 0;

	while (pos < len) {
		switch (str[pos]) {
			case '!':
			case '/':
				LOG_ERROR("fail to parse svf command\n");
				return ERROR_FAIL;
			case '(':
				in_bracket = 1;
				goto parse_char;
			case ')':
				in_bracket = 0;
				goto parse_char;
			default:
parse_char:
				if (!in_bracket && isspace((int) str[pos])) {
					space_found = 1;
					str[pos] = '\0';
				} else if (space_found) {
					argus[num++] = &str[pos];
					space_found = 0;
				}
				break;
		}
		pos++;
	}

	if (num == 0)
		return ERROR_FAIL;

	*num_of_argu = num;

	return ERROR_OK;
}

static int svf_find_string_in_array(char *str, char **strs, int num_of_element)
{
	int i;

	for (i = 0; i < num_of_element; i++) {
		if (!strcmp(str, strs[i]))
			return i;
	}
	return 0xFF;
}

static bool svf_tap_state_is_stable(tap_state_t state)
{
	return aspeed_jtag_tap_is_stable(state);
}

/* helper/binbarybuffer.c */
void *buf_cpy(const void *from, void *_to, unsigned size)
{
	if (NULL == from || NULL == _to)
		return NULL;

	/* copy entire buffer */
	memcpy(_to, from, DIV_ROUND_UP(size, 8));

	/* mask out bits that don't belong to the buffer */
	unsigned trailing_bits = size % 8;
	if (trailing_bits) {
		uint8_t *to = _to;
		to[size / 8] &= (1 << trailing_bits) - 1;
	}
	return _to;
}

static bool buf_cmp_masked(uint8_t a, uint8_t b, uint8_t m)
{
	return (a & m) != (b & m);
}

static bool buf_cmp_trailing(uint8_t a, uint8_t b, uint8_t m, unsigned trailing)
{
	uint8_t mask = (1 << trailing) - 1;
	return buf_cmp_masked(a, b, mask & m);
}

bool buf_cmp(const void *_buf1, const void *_buf2, unsigned size)
{
	if (!_buf1 || !_buf2)
		return _buf1 != _buf2;

	unsigned last = size / 8;
	if (memcmp(_buf1, _buf2, last) != 0)
		return false;

	unsigned trailing = size % 8;
	if (!trailing)
		return false;

	const uint8_t *buf1 = _buf1, *buf2 = _buf2;
	return buf_cmp_trailing(buf1[last], buf2[last], 0xff, trailing);
}

bool buf_cmp_mask(const void *_buf1, const void *_buf2,
	const void *_mask, unsigned size)
{
	if (!_buf1 || !_buf2)
		return _buf1 != _buf2 || _buf1 != _mask;

	const uint8_t *buf1 = _buf1, *buf2 = _buf2, *mask = _mask;
	unsigned last = size / 8;
	for (unsigned i = 0; i < last; i++) {
		if (buf_cmp_masked(buf1[i], buf2[i], mask[i]))
			return true;
	}
	unsigned trailing = size % 8;
	if (!trailing)
		return false;
	return buf_cmp_trailing(buf1[last], buf2[last], mask[last], trailing);
}

void *buf_set_ones(void *_buf, unsigned size)
{
	uint8_t *buf = _buf;
	if (!buf)
		return NULL;

	memset(buf, 0xff, size / 8);

	unsigned trailing_bits = size % 8;
	if (trailing_bits)
		buf[size / 8] = (1 << trailing_bits) - 1;

	return buf;
}

void *buf_set_buf(const void *_src, unsigned src_start,
	void *_dst, unsigned dst_start, unsigned len)
{
	const uint8_t *src = _src;
	uint8_t *dst = _dst;
	unsigned i, sb, db, sq, dq, lb, lq;

	sb = src_start / 8;
	db = dst_start / 8;
	sq = src_start % 8;
	dq = dst_start % 8;
	lb = len / 8;
	lq = len % 8;

	src += sb;
	dst += db;

	/* check if both buffers are on byte boundary and
	 * len is a multiple of 8bit so we can simple copy
	 * the buffer */
	if ((sq == 0) && (dq == 0) &&  (lq == 0)) {
		for (i = 0; i < lb; i++)
			*dst++ = *src++;
		return _dst;
	}

	/* fallback to slow bit copy */
	for (i = 0; i < len; i++) {
		if (((*src >> (sq&7)) & 1) == 1)
			*dst |= 1 << (dq&7);
		else
			*dst &= ~(1 << (dq&7));
		if (sq++ == 7) {
			sq = 0;
			src++;
		}
		if (dq++ == 7) {
			dq = 0;
			dst++;
		}
	}

	return _dst;
}

/*
 * macro is used to print the svf hex buffer at desired debug level
 * DEBUG, INFO, ERROR, USER
 */
#define SVF_BUF_LOG(_lvl, _buf, _nbits, _desc)							\
	svf_hexbuf_print(_lvl,  __FILE__, __LINE__, __func__, _buf, _nbits, _desc)

static void svf_hexbuf_print(int dbg_lvl, const char *file, unsigned line,
							 const char *function, const uint8_t *buf,
							 int bit_len, const char *desc)
{
	int j, len = 0;
	int byte_len = DIV_ROUND_UP(bit_len, 8);
	int msbits = bit_len % 8;

	/* allocate 2 bytes per hex digit */
	char *prbuf = malloc((byte_len * 2) + 2 + 1);
	if (!prbuf)
		return;

	/* print correct number of bytes, mask excess bits where applicable */
	uint8_t msb = buf[byte_len - 1] & (msbits ? (1 << msbits) - 1 : 0xff);
	len = sprintf(prbuf, msbits <= 4 ? "0x%01x" : "0x%02x", msb);
	for (j = byte_len - 2; j >= 0; j--)
		len += sprintf(prbuf + len, "%02x", buf[j]);

	log_printf_lf(dbg_lvl, file, line, function, "%8s = %s\n", desc ? desc : " ", prbuf);

	free(prbuf);
}

static int svf_check_tdo(void)
{
	int i, len, index_var;

	for (i = 0; i < svf_check_tdo_para_index; i++) {
		index_var = svf_check_tdo_para[i].buffer_offset;
		len = svf_check_tdo_para[i].bit_len;
		if ((svf_check_tdo_para[i].enabled)
				&& buf_cmp_mask(&svf_tdi_buffer[index_var], &svf_tdo_buffer[index_var],
				&svf_mask_buffer[index_var], len)) {
			LOG_ERROR("tdo check error at line %d\n",
				svf_check_tdo_para[i].line_num);
			SVF_BUF_LOG(LOG_ERROR, &svf_tdi_buffer[index_var], len, "READ");
			SVF_BUF_LOG(LOG_ERROR, &svf_tdo_buffer[index_var], len, "WANT");
			SVF_BUF_LOG(LOG_ERROR, &svf_mask_buffer[index_var], len, "MASK");
		}
	}
	svf_check_tdo_para_index = 0;

	return ERROR_OK;
}

static int svf_add_check_para(uint8_t enabled, int buffer_offset, int bit_len)
{
	if (svf_check_tdo_para_index >= SVF_CHECK_TDO_PARA_SIZE) {
		LOG_ERROR("toooooo many operation undone");
		return ERROR_FAIL;
	}

	svf_check_tdo_para[svf_check_tdo_para_index].line_num = svf_line_number;
	svf_check_tdo_para[svf_check_tdo_para_index].bit_len = bit_len;
	svf_check_tdo_para[svf_check_tdo_para_index].enabled = enabled;
	svf_check_tdo_para[svf_check_tdo_para_index].buffer_offset = buffer_offset;
	svf_check_tdo_para_index++;

	return ERROR_OK;
}

static int svf_run_command(char *cmd_str)
{
	char *argus[256], command;
	int num_of_argu = 0, i;

	/* tmp variable */
	int i_tmp;

	/* for RUNTEST */
	int run_count;
	int min_time;
	/* for XXR */
	struct svf_xxr_para *xxr_para_tmp;
	uint8_t **pbuffer_tmp;
	scan_field_t field;
	/* for STATE */
	tap_state_t *path = NULL, state;

	if (ERROR_OK != svf_parse_cmd_string(cmd_str, strlen(cmd_str), argus, &num_of_argu))
		return ERROR_FAIL;

	/* NOTE: we're a bit loose here, because we ignore case in
	 * TAP state names (instead of insisting on uppercase).
	 */
	command = svf_find_string_in_array(argus[0],
			(char **)svf_command_name, ARRAY_SIZE(svf_command_name));
	switch (command) {
		/* ENDxR stable_state */
		case ENDDR:
		case ENDIR:
			if (num_of_argu != 2) {
				LOG_ERROR("invalid parameter of %s", argus[0]);
				return ERROR_FAIL;
			}

			i_tmp = tap_state_by_name(argus[1]);

			if (svf_tap_state_is_stable(i_tmp)) {
				if (command == ENDIR) {
					svf_para.ir_end_state = i_tmp;
					LOG_DEBUG("\tIR end_state = %s\n",
							tap_state_name(i_tmp));
				} else {
					svf_para.dr_end_state = i_tmp;
					LOG_DEBUG("\tDR end_state = %s\n",
							tap_state_name(i_tmp));
				}
			} else {
				LOG_ERROR("%s: %s is not a stable state\n",
						argus[0], argus[1]);
				return ERROR_FAIL;
			}
			break;
		/* FREQUENCY [cycles HZ] */
		case FREQUENCY:
			if ((num_of_argu != 1) && (num_of_argu != 3)) {
				LOG_ERROR("invalid parameter of %s", argus[0]);
				return ERROR_FAIL;
			}
			if (1 == num_of_argu) {
				/* TODO: set jtag speed to full speed */
				svf_para.frequency = 0;
			} else {
				if (strcmp(argus[2], "HZ")) {
					LOG_ERROR("HZ not found in FREQUENCY command");
					return ERROR_FAIL;
				}
				svf_para.frequency = atoi(argus[1]);
				/* TODO: set jtag speed to */
				if (svf_para.frequency > 0) {
					
					LOG_DEBUG("\tfrequency = %f\n", svf_para.frequency);
				}
			}
			break;
		case HDR:
			xxr_para_tmp = &svf_para.hdr_para;
			goto XXR_common;
		case HIR:
			xxr_para_tmp = &svf_para.hir_para;
			goto XXR_common;
		case TDR:
			xxr_para_tmp = &svf_para.tdr_para;
			goto XXR_common;
		case TIR:
			xxr_para_tmp = &svf_para.tir_para;
			goto XXR_common;
		case SDR:
			xxr_para_tmp = &svf_para.sdr_para;
			goto XXR_common;
		case SIR:
			xxr_para_tmp = &svf_para.sir_para;
			goto XXR_common;
XXR_common:
			/* XXR length [TDI (tdi)] [TDO (tdo)][MASK (mask)] [SMASK (smask)] */
			if ((num_of_argu > 10) || (num_of_argu % 2)) {
				LOG_ERROR("invalid parameter of %s\n", argus[0]);
				return ERROR_FAIL;
			}
			i_tmp = xxr_para_tmp->len;
			xxr_para_tmp->len = atoi(argus[1]);
			/* If we are to enlarge the buffers, all parts of xxr_para_tmp
			 * need to be freed */
			if (i_tmp < xxr_para_tmp->len) {
				free(xxr_para_tmp->tdi);
				xxr_para_tmp->tdi = NULL;
				free(xxr_para_tmp->tdo);
				xxr_para_tmp->tdo = NULL;
				free(xxr_para_tmp->mask);
				xxr_para_tmp->mask = NULL;
				free(xxr_para_tmp->smask);
				xxr_para_tmp->smask = NULL;
			}

			LOG_DEBUG("\tlength = %d\n", xxr_para_tmp->len);
			xxr_para_tmp->data_mask = 0;
			for (i = 2; i < num_of_argu; i += 2) {
				if ((strlen(argus[i + 1]) < 3) || (argus[i + 1][0] != '(') ||
				(argus[i + 1][strlen(argus[i + 1]) - 1] != ')')) {
					LOG_ERROR("data section error\n");
					return ERROR_FAIL;
				}
				argus[i + 1][strlen(argus[i + 1]) - 1] = '\0';
				/* TDI, TDO, MASK, SMASK */
				if (!strcmp(argus[i], "TDI")) {
					/* TDI */
					pbuffer_tmp = &xxr_para_tmp->tdi;
					xxr_para_tmp->data_mask |= XXR_TDI;
				} else if (!strcmp(argus[i], "TDO")) {
					/* TDO */
					pbuffer_tmp = &xxr_para_tmp->tdo;
					xxr_para_tmp->data_mask |= XXR_TDO;
				} else if (!strcmp(argus[i], "MASK")) {
					/* MASK */
					pbuffer_tmp = &xxr_para_tmp->mask;
					xxr_para_tmp->data_mask |= XXR_MASK;
				} else if (!strcmp(argus[i], "SMASK")) {
					/* SMASK */
					pbuffer_tmp = &xxr_para_tmp->smask;
					xxr_para_tmp->data_mask |= XXR_SMASK;
				} else {
					LOG_ERROR("unknown parameter: %s\n", argus[i]);
					return ERROR_FAIL;
				}
				if (ERROR_OK !=
				svf_copy_hexstring_to_binary(&argus[i + 1][1], pbuffer_tmp, i_tmp,
					xxr_para_tmp->len)) {
					LOG_ERROR("fail to parse hex value\n");
					return ERROR_FAIL;
				}
				SVF_BUF_LOG(LOG_DEBUG, *pbuffer_tmp, xxr_para_tmp->len, argus[i]);
			}
			/* If a command changes the length of the last scan of the same type and the
			 * MASK parameter is absent, */
			/* the mask pattern used is all cares */
			if (!(xxr_para_tmp->data_mask & XXR_MASK) && (i_tmp != xxr_para_tmp->len)) {
				/* MASK not defined and length changed */
				if (ERROR_OK !=
				svf_adjust_array_length(&xxr_para_tmp->mask, i_tmp,
					xxr_para_tmp->len)) {
					LOG_ERROR("fail to adjust length of array\n");
					return ERROR_FAIL;
				}
				buf_set_ones(xxr_para_tmp->mask, xxr_para_tmp->len);
			}
			/* If TDO is absent, no comparison is needed, set the mask to 0 */
			if (!(xxr_para_tmp->data_mask & XXR_TDO)) {
				if (NULL == xxr_para_tmp->tdo) {
					if (ERROR_OK !=
					svf_adjust_array_length(&xxr_para_tmp->tdo, i_tmp,
						xxr_para_tmp->len)) {
						LOG_ERROR("fail to adjust length of array\n");
						return ERROR_FAIL;
					}
				}
				if (NULL == xxr_para_tmp->mask) {
					if (ERROR_OK !=
					svf_adjust_array_length(&xxr_para_tmp->mask, i_tmp,
						xxr_para_tmp->len)) {
						LOG_ERROR("fail to adjust length of array\n");
						return ERROR_FAIL;
					}
				}
				memset(xxr_para_tmp->mask, 0, (xxr_para_tmp->len + 7) >> 3);
			}
			/* do scan if necessary */
			if (SDR == command) {
				/* assemble dr data */
				i = 0;
				buf_set_buf(svf_para.hdr_para.tdi,
						0,
						&svf_tdi_buffer[svf_buffer_index],
						i,
						svf_para.hdr_para.len);
				i += svf_para.hdr_para.len;
				buf_set_buf(svf_para.sdr_para.tdi,
						0,
						&svf_tdi_buffer[svf_buffer_index],
						i,
						svf_para.sdr_para.len);
				i += svf_para.sdr_para.len;
				buf_set_buf(svf_para.tdr_para.tdi,
						0,
						&svf_tdi_buffer[svf_buffer_index],
						i,
						svf_para.tdr_para.len);
				i += svf_para.tdr_para.len;

				/* add check data */
				if (svf_para.sdr_para.data_mask & XXR_TDO) {
					/* assemble dr mask data */
					i = 0;
					buf_set_buf(svf_para.hdr_para.mask,
							0,
							&svf_mask_buffer[svf_buffer_index],
							i,
							svf_para.hdr_para.len);
					i += svf_para.hdr_para.len;
					buf_set_buf(svf_para.sdr_para.mask,
							0,
							&svf_mask_buffer[svf_buffer_index],
							i,
							svf_para.sdr_para.len);
					i += svf_para.sdr_para.len;
					buf_set_buf(svf_para.tdr_para.mask,
							0,
							&svf_mask_buffer[svf_buffer_index],
							i,
							svf_para.tdr_para.len);

					/* assemble dr check data */
					i = 0;
					buf_set_buf(svf_para.hdr_para.tdo,
							0,
							&svf_tdo_buffer[svf_buffer_index],
							i,
							svf_para.hdr_para.len);
					i += svf_para.hdr_para.len;
					buf_set_buf(svf_para.sdr_para.tdo,
							0,
							&svf_tdo_buffer[svf_buffer_index],
							i,
							svf_para.sdr_para.len);
					i += svf_para.sdr_para.len;
					buf_set_buf(svf_para.tdr_para.tdo,
							0,
							&svf_tdo_buffer[svf_buffer_index],
							i,
							svf_para.tdr_para.len);
					i += svf_para.tdr_para.len;

					svf_add_check_para(1, svf_buffer_index, i);
				} else
					svf_add_check_para(0, svf_buffer_index, i);
				field.num_bits = i;
				field.out_value = &svf_tdi_buffer[svf_buffer_index];
				field.in_value = (xxr_para_tmp->data_mask & XXR_TDO) ? &svf_tdi_buffer[svf_buffer_index] : NULL;
				/* NOTE:  doesn't use SVF-specified state paths */
				LOG_DEBUG("dr_scan: num_bits %d end_state %d\n",
					field.num_bits, svf_para.dr_end_state);
				aspeed_jtag_dr_scan(svf_jtag, field.num_bits,
					field.out_value,
					field.in_value,
					svf_para.dr_end_state);
				svf_check_tdo();
			} else if (SIR == command) {
				/* assemble ir data */
				i = 0;
				buf_set_buf(svf_para.hir_para.tdi,
						0,
						&svf_tdi_buffer[svf_buffer_index],
						i,
						svf_para.hir_para.len);
				i += svf_para.hir_para.len;
				buf_set_buf(svf_para.sir_para.tdi,
						0,
						&svf_tdi_buffer[svf_buffer_index],
						i,
						svf_para.sir_para.len);
				i += svf_para.sir_para.len;
				buf_set_buf(svf_para.tir_para.tdi,
						0,
						&svf_tdi_buffer[svf_buffer_index],
						i,
						svf_para.tir_para.len);
				i += svf_para.tir_para.len;

				/* add check data */
				if (svf_para.sir_para.data_mask & XXR_TDO) {
					/* assemble dr mask data */
					i = 0;
					buf_set_buf(svf_para.hir_para.mask,
							0,
							&svf_mask_buffer[svf_buffer_index],
							i,
							svf_para.hir_para.len);
					i += svf_para.hir_para.len;
					buf_set_buf(svf_para.sir_para.mask,
							0,
							&svf_mask_buffer[svf_buffer_index],
							i,
							svf_para.sir_para.len);
					i += svf_para.sir_para.len;
					buf_set_buf(svf_para.tir_para.mask,
							0,
							&svf_mask_buffer[svf_buffer_index],
							i,
							svf_para.tir_para.len);

					/* assemble dr check data */
					i = 0;
					buf_set_buf(svf_para.hir_para.tdo,
							0,
							&svf_tdo_buffer[svf_buffer_index],
							i,
							svf_para.hir_para.len);
					i += svf_para.hir_para.len;
					buf_set_buf(svf_para.sir_para.tdo,
							0,
							&svf_tdo_buffer[svf_buffer_index],
							i,
							svf_para.sir_para.len);
					i += svf_para.sir_para.len;
					buf_set_buf(svf_para.tir_para.tdo,
							0,
							&svf_tdo_buffer[svf_buffer_index],
							i,
							svf_para.tir_para.len);
					i += svf_para.tir_para.len;

					svf_add_check_para(1, svf_buffer_index, i);
				} else
					svf_add_check_para(0, svf_buffer_index, i);
				field.num_bits = i;
				field.out_value = &svf_tdi_buffer[svf_buffer_index];
				field.in_value = (xxr_para_tmp->data_mask & XXR_TDO) ? &svf_tdi_buffer[svf_buffer_index] : NULL;
				/* NOTE:  doesn't use SVF-specified state paths */
				LOG_DEBUG("ir_scan: num_bits %d end_state %d\n",
					field.num_bits, svf_para.ir_end_state);
				aspeed_jtag_ir_scan(svf_jtag, field.num_bits,
					field.out_value,
					field.in_value,
					svf_para.ir_end_state);
				svf_check_tdo();
			}
			break;
		case PIO:
		case PIOMAP:
			LOG_ERROR("PIO and PIOMAP are not supported");
			return ERROR_FAIL;
		case RUNTEST:
			/* RUNTEST [run_state] run_count run_clk [min_time SEC [MAXIMUM max_time
			 * SEC]] [ENDSTATE end_state] */
			/* RUNTEST [run_state] min_time SEC [MAXIMUM max_time SEC] [ENDSTATE
			 * end_state] */
			if ((num_of_argu < 3) || (num_of_argu > 11)) {
				LOG_ERROR("invalid parameter of %s", argus[0]);
				return ERROR_FAIL;
			}
			/* init */
			run_count = 0;
			min_time = 0;
			i = 1;

			/* run_state */
			i_tmp = tap_state_by_name(argus[i]);
			if (i_tmp != TAP_INVALID) {
				if (svf_tap_state_is_stable(i_tmp)) {
					svf_para.runtest_run_state = i_tmp;

					/* When a run_state is specified, the new
					 * run_state becomes the default end_state.
					 */
					svf_para.runtest_end_state = i_tmp;
					LOG_DEBUG("\trun_state = %s", tap_state_name(i_tmp));
					i++;
				} else {
					LOG_ERROR("%s: %s is not a stable state", argus[0], tap_state_name(i_tmp));
					return ERROR_FAIL;
				}
			}

			/* run_count run_clk */
			if (((i + 2) <= num_of_argu) && strcmp(argus[i + 1], "SEC")) {
				if (!strcmp(argus[i + 1], "TCK")) {
					/* clock source is TCK */
					run_count = atoi(argus[i]);
					LOG_DEBUG("\trun_count@TCK = %d\n", run_count);
				} else {
					LOG_ERROR("%s not supported for clock\n", argus[i + 1]);
					return ERROR_FAIL;
				}
				i += 2;
			}
			/* min_time SEC */
			if (((i + 2) <= num_of_argu) && !strcmp(argus[i + 1], "SEC")) {
				min_time = svf_atoi_s2ms(argus[i]);
				LOG_DEBUG("\tmin_time = %d %s s\n", min_time, argus[i]);
				i += 2;
			}
			/* MAXIMUM max_time SEC */
			if (((i + 3) <= num_of_argu) &&
			!strcmp(argus[i], "MAXIMUM") && !strcmp(argus[i + 2], "SEC")) {
				int max_time = 0;
				max_time = svf_atoi_s2ms(argus[i + 1]);
				LOG_DEBUG("\tmax_time = %ds\n", max_time);
				i += 3;
			}
			/* ENDSTATE end_state */
			if (((i + 2) <= num_of_argu) && !strcmp(argus[i], "ENDSTATE")) {
				i_tmp = tap_state_by_name(argus[i + 1]);

				if (svf_tap_state_is_stable(i_tmp)) {
					svf_para.runtest_end_state = i_tmp;
					LOG_DEBUG("\tend_state = %s\n", tap_state_name(i_tmp));
				} else {
					LOG_ERROR("%s: %s is not a stable state\n", argus[0], tap_state_name(i_tmp));
					return ERROR_FAIL;
				}
				i += 2;
			}

			/* all parameter should be parsed */
			if (i == num_of_argu) {
				/* FIXME handle statemove failures */
				uint32_t min_usec = min_time;

				/* enter into run_state if necessary */
				aspeed_jtag_set_tap_state(svf_jtag, svf_para.runtest_run_state);

				/* add clocks and/or min wait */
				if (run_count > 0) {
					aspeed_jtag_run_test(svf_jtag, run_count);
				}

				if (min_usec > 0) {
					if (min_usec >= 1000)
						log_debug("wait: %d ms\n", min_usec);
					osDelay(min_usec);
				}

				/* move to end_state if necessary */
				if (svf_para.runtest_end_state != svf_para.runtest_run_state)
					aspeed_jtag_set_tap_state(svf_jtag, svf_para.runtest_end_state);
			} else {
				LOG_ERROR("fail to parse parameter of RUNTEST, %d out of %d is parsed\n",
						i,
						num_of_argu);
				return ERROR_FAIL;
			}
			break;
		case STATE:
			/* STATE [pathstate1 [pathstate2 ...[pathstaten]]] stable_state */
			if (num_of_argu < 2) {
				LOG_ERROR("invalid parameter of %s\n", argus[0]);
				return ERROR_FAIL;
			}
			if (num_of_argu > 2) {
				/* STATE pathstate1 ... stable_state */
				path = malloc((num_of_argu - 1) * sizeof(tap_state_t));
				if (NULL == path) {
					LOG_ERROR("not enough memory\n");
					return ERROR_FAIL;
				}
				num_of_argu--;	/* num of path */
				i_tmp = 1;		/* path is from parameter 1 */
				for (i = 0; i < num_of_argu; i++, i_tmp++) {
					path[i] = tap_state_by_name(argus[i_tmp]);
					if (path[i] == TAP_INVALID) {
						LOG_ERROR("%s: %s is not a valid state\n", argus[0], argus[i_tmp]);
						free(path);
						return ERROR_FAIL;
					}
				}
				if (num_of_argu > 0) {
					/* execute last path if necessary */
					if (svf_tap_state_is_stable(path[num_of_argu - 1])) {
						/* last state MUST be stable state */
						aspeed_jtag_set_tap_state(svf_jtag, path[num_of_argu - 1]);
						LOG_DEBUG("\tmove to %s by path_move\n",
								tap_state_name(path[num_of_argu - 1]));
					} else {
						LOG_ERROR("%s: %s is not a stable state\n",
								argus[0],
								tap_state_name(path[num_of_argu - 1]));
						free(path);
						return ERROR_FAIL;
					}
				}

				free(path);
				path = NULL;
			} else {
				/* STATE stable_state */
				state = tap_state_by_name(argus[1]);
				if (svf_tap_state_is_stable(state)) {
					LOG_DEBUG("\tmove to %s by aspeed_jtag_set_tap_state\n",
							tap_state_name(state));
					/* FIXME handle statemove failures */
					aspeed_jtag_set_tap_state(svf_jtag, state);
				} else {
					LOG_ERROR("%s: %s is not a stable state\n",
							argus[0], tap_state_name(state));
					return ERROR_FAIL;
				}
			}
			break;
		default:
			LOG_ERROR("invalid svf command: %s\n", argus[0]);
			return ERROR_FAIL;
	}
	return ERROR_OK;
}

static char svf_command_buffer[1024];
static uint32_t cmd_pos;
static int svf_read_command(char *svf_read_line, uint32_t *cmd_pos)
{
	unsigned char ch;
	int i = 0;
	int cmd_ok = 0, slash = 0, comment = 0;

	for (i=0; i < strlen(svf_read_line);i++){
		ch = svf_read_line[i];
		switch (ch) {
			case '!':
				slash = 0;
				comment = 1;
				svf_line_number++;
				break;
			case '/':
				if (++slash == 2) {
					slash = 0;
					svf_line_number++;
				}
				break;
			case ';':
				slash = 0;
				cmd_ok = 1;
				break;
			case '\n':
				svf_line_number++;
				/* fallthrough */
			case '\r':
				slash = 0;
				/* Don't save '\r' and '\n'*/
				if (*cmd_pos)
					break;
				/* fallthrough */
			default:
				/* insert a space before '(' */
				if ('(' == ch)
					svf_command_buffer[(*cmd_pos)++] = ' ';

				svf_command_buffer[(*cmd_pos)++] = ch;

				/* insert a space after ')' */
				if (')' == ch)
					svf_command_buffer[(*cmd_pos)++] = ' ';
				break;
		}
		if (comment) {
			log_debug("%s\n", svf_read_line);
			break;
		}
	}
	if (cmd_ok) {
		svf_command_buffer[(*cmd_pos)] = '\0';
		return 0;
	} else if (comment) {
		return 1;
	} else {
		return 2;
	}
}

#define ASCII_CR 0x0D
#define svfMAX_INPUT_SIZE 1024
static const CLI_Command_Definition_t svf_cmd;
static osThreadId_t tid_svf_task;
static osThreadAttr_t tattr_svf_task;
static osEventFlagsId_t svf_task_event;
static uint8_t *rx_buff, *tx_buff;
static void svf_task(void *argv)
{
	int rx_len, wait_ret, snlen;
	uint32_t ucInputIndex = 0;
	signed char cRxedChar;
	static char cInputString[svfMAX_INPUT_SIZE];
	uint8_t cr = 0, ret;
	svf_check_tdo_para_index = 0;
	svf_line_number = 0;
	svf_check_tdo_para = malloc(sizeof(struct svf_check_tdo_para) * SVF_CHECK_TDO_PARA_SIZE);
	svf_tdi_buffer = malloc(SVF_MAX_BUFFER_SIZE_TO_COMMIT);
	svf_tdo_buffer = malloc(SVF_MAX_BUFFER_SIZE_TO_COMMIT);
	svf_mask_buffer = malloc(SVF_MAX_BUFFER_SIZE_TO_COMMIT);
	rx_buff = pvPortMallocNc(1024);
	tx_buff = pvPortMallocNc(2);
	tx_buff[0] = '>';
	snlen = 1;
	memcpy(&svf_para, &svf_para_init, sizeof(svf_para));
	usb_acquire(&usb[0]);
	for (;;) {
		rx_len = usb_read(&usb[0], 2, rx_buff, 1024);
		for(int i = 0; i < rx_len; i++) {
			cRxedChar = rx_buff[i];
			switch (cRxedChar) {
				case ASCII_CR: // Enter key pressed
					cInputString[ucInputIndex] = '\r';
					cInputString[ucInputIndex + 1] = '\n';
					cInputString[ucInputIndex + 2] = '\0';
					cr = 1;
					ucInputIndex = 0;
					break;
				case '\n':
					break;
				default:
					cInputString[ucInputIndex] = cRxedChar;
					ucInputIndex++;
					break;
			}
			if (cr) {
				ret = svf_read_command(cInputString, &cmd_pos);
				if (ret == 0) {
					log_debug("%s\n", svf_command_buffer);
					svf_run_command(svf_command_buffer);
					usb_write(&usb[0], 1, (uint8_t *)tx_buff, snlen);
					cmd_pos = 0;
				} else if (ret == 1){
					usb_write(&usb[0], 1, (uint8_t *)tx_buff, snlen);
				}
				memset(cInputString, 0, svfMAX_INPUT_SIZE);
				cr = 0;
			}
		}
		wait_ret = osEventFlagsWait(svf_task_event, 0x00000001U, osFlagsWaitAll, 1);
		if (wait_ret == 1) {
			usb_release(&usb[0]);
			free(svf_check_tdo_para);
			free(svf_tdi_buffer);
			free(svf_tdo_buffer);
			free(svf_mask_buffer);
			vPortFreeNc(rx_buff);
			vPortFreeNc(tx_buff);
			osThreadTerminate(tid_svf_task);
		}
	}
}

static void svf_cmd_handler(int argc, char *argv[])
{
	char option;
	int chip = 0;
	bool run = 0, delete = 0;
	optind = 0;
	while ((option = getopt(argc, argv, "c:hrd")) != (char)-1) {
		switch (option) {
			case 'c':
				chip = atoi(optarg);
				break;
			case 'h':
			case '?':
				printf("%s", svf_cmd.pcHelpString);
				return;
			case 'r':
				run = 1;
				break;
			case 'd':
				delete = 1;
				break;
			default:
				log_warn("unknown option -%c", option);
				break;
			}
	}
	svf_jtag = &jtag[chip];
	if (run) {
		svf_task_event = osEventFlagsNew(NULL);
		tattr_svf_task.name = "demo_svf";
		tattr_svf_task.priority = osPriorityBelowNormal;
		tattr_svf_task.stack_size = 0x1000;

		tid_svf_task = osThreadNew(svf_task, NULL, &tattr_svf_task);
	} else if (delete) {
		if (tid_svf_task)
			osEventFlagsSet(svf_task_event, 0x00000001U);
	}

}

CLI_FUNC_DECL(svf, svf_cmd_handler);

static const CLI_Command_Definition_t svf_cmd = 
{
	"svf",
	"\r\nsvf:\n \
	usage: \r\n \
		[-c <jtag device id>]: Use jtag device 0 or 1\r\n \
		[-h]: Show this message\r\n \
		[-r]: Run svf daemon\r\n \
		[-d]: Delete svf daemon \r\n",
	CLI_FUNC_SYM(svf),
	-1
};

void demo_svf_init(void)
{
	FreeRTOS_CLIRegisterCommand(&svf_cmd);
}