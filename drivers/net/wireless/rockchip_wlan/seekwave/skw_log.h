/* SPDX-License-Identifier: GPL-2.0 */
/******************************************************************************
 *
 * Copyright(c) 2020-2030  Seekwave Corporation.
 *
 *****************************************************************************/
#ifndef __SKW_LOG_H__
#define __SKW_LOG_H__

#define SKW_ERROR    BIT(0)
#define SKW_WARN     BIT(1)
#define SKW_INFO     BIT(2)
#define SKW_DEBUG    BIT(3)
#define SKW_DETAIL   BIT(4)

#define SKW_CMD      BIT(16)
#define SKW_EVENT    BIT(17)
#define SKW_SCAN     BIT(18)
#define SKW_TIMER    BIT(19)
#define SKW_STATE    BIT(20)
#define SKW_WORK     BIT(21)

#define SKW_DUMP     BIT(31)

unsigned long skw_log_level(void);

#define skw_data_path_log(fmt, ...) \
	do { \
		if ((skw_log_level() & SKW_DEBUG)) \
			printk_ratelimited("[SKWIFI DATA PATH] %s: "fmt, __func__, ##__VA_ARGS__); \
	} while (0)

#define skw_log(level, fmt, ...) \
	do { \
		if (skw_log_level() & level) \
			pr_err(fmt,  ##__VA_ARGS__); \
	} while (0)

#define skw_err(fmt, ...) \
	skw_log(SKW_ERROR, "[SKWIFI ERROR] %s: "fmt, __func__, ##__VA_ARGS__)

#define skw_warn(fmt, ...) \
	skw_log(SKW_WARN, "[SKWIFI WARN] %s: "fmt, __func__, ##__VA_ARGS__)

#define skw_info(fmt, ...) \
	skw_log(SKW_INFO, "[SKWIFI INFO] %s: "fmt, __func__, ##__VA_ARGS__)

#define skw_dbg(fmt, ...) \
	skw_log(SKW_DEBUG, "[SKWIFI DBG] %s: "fmt, __func__, ##__VA_ARGS__)

#define skw_detail(fmt, ...) \
	skw_log(SKW_DETAIL, "[SKWIFI DETAIL] %s: "fmt, __func__, ##__VA_ARGS__)

#define skw_hex_dump(prefix, buf, len, force) \
	do { \
		if ((skw_log_level() & SKW_DUMP) || force) { \
			u8 str[32] = {0};  \
			snprintf(str, sizeof(str), "[SKWIFI DUMP] %s", prefix); \
			print_hex_dump(KERN_ERR, str, \
				DUMP_PREFIX_OFFSET, 16, 1, buf, len, true); \
		} \
	} while (0)

void skw_log_level_init(void);
#endif
