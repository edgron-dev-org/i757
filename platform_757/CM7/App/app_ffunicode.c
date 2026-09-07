/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_ffunicode.c — minimal FatFs LFN unicode hooks (ASCII passthrough)
 *
 * _USE_LFN >= 1 makes ff.c reference ff_convert()/ff_wtoupper(), normally provided by the
 * FatFs option file ccsbcs.c with full OEM code-page tables. Every file this firmware
 * creates on the card is plain ASCII (datalog daily CSVs, i757.cfg), so instead of ~500
 * lines of CP850 tables we pass ASCII through and reject anything else: a non-ASCII name
 * fails cleanly with FR_INVALID_NAME instead of being silently mis-mapped. */
#include "ff.h"

#if _USE_LFN != 0

WCHAR ff_convert(WCHAR chr, UINT dir)   /* OEM <-> Unicode, both directions */
{
  (void)dir;
  return (chr < 0x80U) ? chr : 0U;      /* 0 = unconvertible -> FatFs rejects the name */
}

WCHAR ff_wtoupper(WCHAR chr)
{
  return ((chr >= 'a') && (chr <= 'z')) ? (WCHAR)(chr - 0x20U) : chr;
}

#endif
