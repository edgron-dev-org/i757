/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_hsdi.c — CLI view of the onboard HSDI channels (user file).
 *
 * The channels themselves are owned by CM4 (timers TIM1/TIM3/TIM8 + EXTI); this file only
 * reads what CM4 published into the process image. The former CM7-side TIM4->TIM1 pulse
 * self-test is retired: TIM1 now belongs to CM4 as an encoder, so CM7 must not touch it.
 * Contract: docs/HSDI_Configuration_and_Counting.md.
 */
#include "app_hsdi.h"
#include <stdio.h>
#include "app_platform.h"

static const char *mode_str(uint8_t m)
{
  static const char *n[] = { "off", "DI", "cnt-sw", "cnt-hw", "encA", "encB", "encZ" };
  return (m <= APP_HSDI_ENC_Z) ? n[m] : "?";
}

void app_hsdi_cli(void)
{
  extern uint8_t app_hsdi_mode_of(uint8_t ch);
  printf("HSDI (owned by CM4; counts/positions come from the process image)\n\r");
  for (uint8_t ch = 0; ch < 8U; ch++)
  {
    printf("  HSDI%u %-7s level=%u  count=%lu\n\r", (unsigned)ch,
           mode_str(app_hsdi_mode_of(ch)), (unsigned)app_hsdi_level(ch),
           (unsigned long)app_hsdi_count(ch));
  }
  for (uint8_t e = 0; e < 2U; e++)
  {
    printf("  encoder%u pos=%ld revs=%lu zlatch=%lu\n\r", (unsigned)e,
           (long)app_hsdi_position(e), (unsigned long)app_hsdi_revs(e),
           (unsigned long)app_hsdi_zlatch(e));
  }
  printf("  (external Modbus: levels FC02 0x2000.., counts FC04 0x2002.., reset FC05 0x2000..)\n\r");
}
