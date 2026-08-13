#pragma once
// SPDX-License-Identifier: Unlicense
/*
 * Shared between the three translation units of this component: the transport
 * and routing in web_server.cpp, and the two page renderers.
 */
#include "web_server.h"
#include <stddef.h>

extern const char* WEB_TAG;

// Request and response staging. File scope rather than stack: this all runs in
// the NTP task, whose stack has no room for kilobyte buffers.
extern char g_req[3072];
extern char g_resp[26624];
extern const int hdrReserve;

// strcasestr is a GNU extension; spell it out so the build does not depend on
// which libc variant is configured.
const char* ci_find(const char* hay, const char* needle);
