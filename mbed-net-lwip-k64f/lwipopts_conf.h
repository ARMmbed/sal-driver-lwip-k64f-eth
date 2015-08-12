/*
 * Copyright (c) 2012-2015, ARM Limited, All Rights Reserved
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

#ifndef LWIPOPTS_CONF_H
#define LWIPOPTS_CONF_H

#include "k64f_emac_config.h"

#define LWIP_TRANSPORT_ETHERNET       1
#define ETH_PAD_SIZE                  2

#define MEM_SIZE                      (ENET_RX_RING_LEN * (ENET_ETH_MAX_FLEN + RX_BUF_ALIGNMENT) + ENET_TX_RING_LEN * ENET_ETH_MAX_FLEN)

#endif
