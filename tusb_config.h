// SPDX-License-Identifier: MIT
/*
 * Copyright (c) 2021 Álvaro Fernández Rojas <noltari@gmail.com>
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 * Copyright (c) 2020 Damien P. George
 */

#if !defined(_TUSB_CONFIG_H_)
#define _TUSB_CONFIG_H_

#include <tusb_option.h>

#define CFG_TUSB_RHPORT0_MODE OPT_MODE_DEVICE

#define CFG_TUD_CDC 5
#define CFG_TUD_CDC_RX_BUFSIZE 5120  // no harm making these bigger I guess
#define CFG_TUD_CDC_TX_BUFSIZE 5120

#endif /* _TUSB_CONFIG_H_ */
