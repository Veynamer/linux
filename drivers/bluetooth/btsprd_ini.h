/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2026
 */
#ifndef __BTSPRD_INI_H__
#define __BTSPRD_INI_H__

#include <linux/types.h>

int btsprd_ini_pack(const char *ini, size_t ini_len, u8 *out, size_t out_max);

#endif /* __BTSPRD_INI_H__ */
