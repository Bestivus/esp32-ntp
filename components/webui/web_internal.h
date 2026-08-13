#pragma once
// SPDX-License-Identifier: Unlicense
#include "web_server.h"
#include <stddef.h>

extern char g_req[3072];
extern char g_resp[26624];
extern const int hdrReserve;

