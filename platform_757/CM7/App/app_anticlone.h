/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_anticlone.h — anti-clone challenge-response (user file)
 * At firmware startup/runtime, verifies "this hardware holds the genuine private key": issue a random challenge -> SE signs -> verify with embedded public key.
 * Together with RDP2 (lock firmware unreadable) + 608B (private key uncopyable), the trio forms anti-cloning, see docs/Production_Security_Checklist.md */
#ifndef APP_ANTICLONE_H
#define APP_ANTICLONE_H

#define APP_AC_GENUINE   0   /* Genuine hardware (signature verified) */
#define APP_AC_CLONE     1   /* Clone suspected (SE present but signature fails verification) */
#define APP_AC_ABSENT    2   /* No SE (not fitted, or not provisioned) */

int app_anticlone_verify(void);   /* Perform one challenge-response, return one of the above */

#endif
