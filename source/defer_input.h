/*
 * K64F EMAC lwIP driver
 * Copyright (c) 2015, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SAL_DRIVER_LWIP_K64F_SOURCE_DEFER_INPUT_H
#define SAL_DRIVER_LWIP_K64F_SOURCE_DEFER_INPUT_H
#include "lwip/pbuf.h"
#include "netif/etharp.h"
#ifdef __cplusplus
extern "C" {
#endif
void defer_input(struct pbuf *p, struct netif *netif);
void defer_link(struct netif *netif);
#ifdef __cplusplus
}
#endif

#endif // SAL_DRIVER_LWIP_K64F_SOURCE_DEFER_INPUT_H
