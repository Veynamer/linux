// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026
 *
 * Parser for the Unisoc Bluetooth pskey / RF-config INI files.
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include "btsprd_ini.h"

/* Widest field is feature_set / g_LEPowerValue (16 tokens). Leave headroom. */
#define BT_INI_MAX_TOKENS 64

/* Parse a hex ("0x..") or decimal integer starting at @s, stopping at the
 * first non-digit. Bounded by @end.
 */
static u32 bt_ini_token(const char *s, const char *end)
{
	u32 v = 0;
	int base = 10;

	while (s < end && (*s == ' ' || *s == '\t'))
		s++;
	if (s + 1 < end && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
		base = 16;
		s += 2;
	}
	while (s < end) {
		char c = *s;
		int d;

		if (c >= '0' && c <= '9')
			d = c - '0';
		else if (base == 16 && c >= 'a' && c <= 'f')
			d = c - 'a' + 10;
		else if (base == 16 && c >= 'A' && c <= 'F')
			d = c - 'A' + 10;
		else
			break;
		v = v * base + d;
		s++;
	}
	return v;
}

/* Find @needle within [s, end); return the position just past it, or NULL. */
static const char *bt_ini_find(const char *s, const char *end,
			       const char *needle)
{
	size_t nl = strlen(needle);

	while (s + nl <= end) {
		if (!memcmp(s, needle, nl))
			return s + nl;
		s++;
	}
	return NULL;
}

/* Emit the currently-accumulated field: L bytes total, split evenly across
 * @ntok tokens, each little-endian. @cur_L < 0 means no field is pending yet.
 */
static int bt_ini_flush(int cur_L, const u32 *toks, int ntok,
			u8 *out, size_t out_max, size_t *out_pos)
{
	int bpt, i, b;

	if (cur_L < 0)
		return 0;
	if (ntok == 0 || cur_L % ntok) {
		pr_err("btsprd_ini: field L=%d not divisible by %d tokens\n",
		       cur_L, ntok);
		return -EINVAL;
	}
	bpt = cur_L / ntok;
	if (bpt < 1 || bpt > 4) {
		pr_err("btsprd_ini: bad bytes-per-token %d (L=%d, ntok=%d)\n",
		       bpt, cur_L, ntok);
		return -EINVAL;
	}
	if (*out_pos + cur_L > out_max)
		return -EOVERFLOW;

	for (i = 0; i < ntok; i++) {
		u32 v = toks[i];

		for (b = 0; b < bpt; b++)
			out[(*out_pos)++] = (v >> (8 * b)) & 0xff;
	}
	return 0;
}

int btsprd_ini_pack(const char *ini, size_t ini_len, u8 *out, size_t out_max)
{
	const char *p = ini;
	const char *end = ini + ini_len;
	u32 toks[BT_INI_MAX_TOKENS];
	size_t out_pos = 0;
	int total_len = -1;
	int cur_L = -1;
	int ntok = 0;
	int rc;

	while (p < end) {
		const char *le = memchr(p, '\n', end - p);
		const char *s = p;
		const char *eq, *v;

		if (!le)
			le = end;
		p = (le < end) ? le + 1 : end;

		while (s < le && (*s == ' ' || *s == '\t' || *s == '\r'))
			s++;
		if (s == le)
			continue;

		if (*s == '#') {
			const char *t;

			if (total_len < 0) {
				t = bt_ini_find(s, le, "Total Length=");
				if (t)
					total_len = (int)bt_ini_token(t, le);
			}
			t = bt_ini_find(s, le, "/L=");
			if (t) {
				rc = bt_ini_flush(cur_L, toks, ntok, out,
						  out_max, &out_pos);
				if (rc)
					return rc;
				cur_L = (int)bt_ini_token(t, le);
				ntok = 0;
			}
			continue;
		}

		eq = memchr(s, '=', le - s);
		if (!eq)
			continue;

		for (v = eq + 1; v < le; ) {
			const char *te;

			while (v < le && (*v == ' ' || *v == '\t' ||
					  *v == '\r' || *v == ','))
				v++;
			if (v >= le)
				break;
			te = v;
			while (te < le && *te != ',' && *te != ' ' &&
			       *te != '\t' && *te != '\r')
				te++;
			if (ntok >= BT_INI_MAX_TOKENS)
				return -EOVERFLOW;
			toks[ntok++] = bt_ini_token(v, te);
			v = te;
		}
	}

	rc = bt_ini_flush(cur_L, toks, ntok, out, out_max, &out_pos);
	if (rc)
		return rc;

	if (total_len < 0) {
		pr_err("btsprd_ini: no [Total Length=N] header found\n");
		return -EINVAL;
	}
	if ((int)out_pos != total_len) {
		pr_err("btsprd_ini: packed %zu bytes, header says %d\n",
		       out_pos, total_len);
		return -EINVAL;
	}
	return (int)out_pos;
}
