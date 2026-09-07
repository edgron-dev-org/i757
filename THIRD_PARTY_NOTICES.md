# Third-Party Notices

This SDK includes the following third-party components. Each is licensed
under its own terms, which prevail over the SDK LICENSE for that component.
When you distribute a product built with this SDK, include these notices
(this file satisfies the attribution requirements for binary distribution).

| Component | License | License text |
|---|---|---|
| STM32H7xx HAL / LL drivers | BSD-3-Clause (STMicroelectronics) | `platform_757/Drivers/STM32H7xx_HAL_Driver/LICENSE.txt` |
| CMSIS | Apache-2.0 (Arm) | `platform_757/Drivers/CMSIS/LICENSE.txt` |
| FreeRTOS kernel | MIT (Amazon.com) | `platform_757/Middlewares/Third_Party/FreeRTOS/Source/LICENSE` |
| lwIP | BSD-3-Clause (Swedish Institute of Computer Science) | license text carried in each source file header (e.g. `platform_757/Middlewares/Third_Party/LwIP/src/include/lwip/opt.h`) |
| Mbed TLS | Apache-2.0 (Arm / Mbed TLS contributors) | `platform_757/Middlewares/Third_Party/mbedTLS/LICENSE` |
| OpenAMP + libmetal | BSD-3-Clause (Xilinx et al.) | license text carried in source file headers (`platform_757/Middlewares/Third_Party/OpenAMP/`) |
| littlefs | BSD-3-Clause (Arm) | `platform_757/CM7/App/littlefs/LICENSE.md` |
| FatFs | FatFs license, BSD-style (ChaN) | header of `platform_757/Middlewares/Third_Party/FatFs/src/ff.h` |
| STM32 USB Device Library | **SLA0044** (STMicroelectronics — use permitted on ST devices only) | `platform_757/Middlewares/ST/STM32_USB_Device_Library/LICENSE.txt` |
| STM32CubeMX-generated code (`Core/`, peripheral init) | STMicroelectronics terms as stated in each file header | per-file headers |

Notes:

- The SLA0044-licensed USB library restricts use to STMicroelectronics
  devices; this SDK targets the STM32H757, so normal use complies. Porting
  this SDK to non-ST silicon would require replacing that component.
- Edgron-authored files are marked with
  `SPDX-License-Identifier: LicenseRef-Edgron-Source-Available` and are governed by the
  SDK `LICENSE` file.
