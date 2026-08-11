# esp32-ntp Firmware: Troubleshooting Summary & Change Log

**Purpose of this document**: a complete record of every firmware-side issue found and fixed while
bringing up [dniminenn/esp32-ntp](https://github.com/dniminenn/esp32-ntp) on this specific
hardware, migrating it to ESP-IDF v6.0.2, and getting it to a stable, GPS-locked, Stratum-1
working state. Intended as context for continued work in Claude Code — explains *why* the current
source differs from upstream, not just *what* differs. Forward-looking plans (deployment,
upstream PR, further validation) are tracked separately in the local-only `PLANS.local.md`, not
here — this file is the historical/technical record.

**Current status: working.** GPS-locked, Stratum 1, sub-microsecond internal precision (7ns PPS
jitter, matching/beating the maintainer's own published benchmarks), 12+ hours of clean uptime
with zero watchdog resets, zero W5500 chip resets, zero PPS rejects. MQTT status publishing to
Home Assistant (Part 6) also confirmed working end-to-end against a real EMQX broker.

---

## Hardware reference (for context, not something that needs fixing)

- **MCU**: "D1 Mini ESP32" board (ESP32-WROOM-32 module, no mounting holes, held in a 3D-printed
  friction-fit cradle)
- **Ethernet**: WIZnet W5500 on SPI2/HSPI — CS=GPIO25, MOSI=GPIO33, MISO=GPIO35, SCLK=GPIO32,
  INT=GPIO34, RST=GPIO26
- **GPS**: SparkFun MAX-M10S — UART2, TX(GPS)→GPIO16, RX(GPS)←GPIO17 (unused), PPS→GPIO19
  - **Confirmed baud: 9600** (not the 38400 SparkFun documents as default — this specific module's
    saved config differs from factory default; already correctly hardcoded in `config.cpp`, not a
    bug)
- Network: static IP planned for final deployment (10.100.0.40, Lab VLAN); currently burning in
  on DHCP on a different subnet (10.10.0.0/24) for testing, since that's the only reachable
  network with console access

All wiring independently validated working via standalone Arduino test sketches before any
ESP-IDF firmware work began — hardware was never in question at any point in the debugging below.

---

## Part 1: Why the IDF version migration happened at all

**Original symptom on ESP-IDF v5.5.5** (the version initially installed): firmware built and ran
cleanly, W5500/Ethernet/DHCP worked, GPS NMEA parsing worked (valid fix, correct lat/long), but
**`ntp_pps_count` stayed at 0 forever** — the MCPWM hardware capture callback for the PPS pin never
fired, so the device could never achieve GPS lock or serve as Stratum 1.

**Root cause hypothesis**: the project's own source has a comment next to the MCPWM capture
channel config noting a struct field "was also removed... in IDF v6," direct evidence the code is
version-sensitive. The README states the project is tested against a v6.0-dev toolchain, not the
v5.5.5 that was initially installed.

**Decision**: migrate to ESP-IDF v6.0.2 (a stable release in the same v6.0 line) to test this
hypothesis. **Confirmed correct** — after resolving the build/runtime issues below, GPS lock and
PPS capture both work cleanly under v6.0.2.

---

## Part 2: Build errors fixed to get v6.0.2 compiling

ESP-IDF v6.0 made two categories of breaking change relevant here: it reorganized driver
components (many `esp_driver_*`/legacy `driver` components split or renamed), and it made several
compiler warnings into hard errors by default. Eight distinct fixes were needed, found by fixing
one error, rebuilding, and repeating — each confirmed against the actual v6.0.2 ESP-IDF source
rather than guessed.

| # | File | Error | Root cause | Fix |
|---|---|---|---|---|
| 1 | top-level `CMakeLists.txt` | `missing initializer for field '_read_data_buf'` in vendored `wizchip_conf.c` | Newer GCC stricter about partial struct initializers (C still auto-zeros unlisted fields — cosmetic, not a real bug) | Added `idf_build_set_property(COMPILE_OPTIONS "-Wno-missing-field-initializers" APPEND)` |
| 2 | `components/w5500_eth/CMakeLists.txt` | `driver/gpio.h: No such file or directory` | GPIO moved to `esp_driver_gpio` component in v6.0; generic `driver` component only depends on it via `PRIV_REQUIRES` (not passed to dependents) | Added `esp_driver_gpio` to `REQUIRES` |
| 3 | `components/w5500_eth/w5500_eth.cpp` | `'++' expression of 'volatile'-qualified type is deprecated` (5 locations: lines ~90, 114, 213, 276, 317) | C++20+ deprecates compound/increment operators on `volatile` variables; project builds with `-std=gnu++26` | Rewrote each as plain read-then-assign, e.g. `g_w5k_txns = g_w5k_txns + 1;` |
| 4 | `components/gps/CMakeLists.txt` | Same `driver/gpio.h` error, different component | Same as #2 | Added `esp_driver_gpio` to `REQUIRES` |
| 5 | `components/gps/CMakeLists.txt` | `driver/uart.h: No such file or directory` | UART moved to `esp_driver_uart` component in v6.0 | Added `esp_driver_uart` to `REQUIRES` (v6.0.2's error message literally named the exact fix) |
| 6 | `components/ntp_stats/ntp_stats.cpp` | `'SOCK_STREAM'/'SOCK_DGRAM' redefined` | WIZnet's `w5500.h` `#define`s these as aliases for its own constants with no include guard, colliding with lwIP's real POSIX values — and this file legitimately needs the real value for its own `socket()` call | `#pragma push_macro`/`pop_macro` guard: `#undef` both macros immediately before including the WIZnet-touching header, restore immediately after |
| 7 | Linker: `multiple definition of 'close'` | WIZnet's `socket.c` defines its own `close()`, colliding with the real POSIX `close()` once a real network socket (stats HTTP server) and WIZnet's raw socket API coexist in one binary | Traced every call site first (to avoid silently rebinding the wrong `close()` at runtime): `ioLibrary_Driver/Ethernet/socket.c` (definition), `ioLibrary_Driver/Internet/DHCP/dhcp.c`, `components/w5k/w5k_tcp_wrapper.c`, `components/w5k/w5k_udp_wrapper.c` | `set_source_files_properties(... COMPILE_DEFINITIONS "close=w5k_wiz_close")` scoped to exactly those four files, in both `w5500_eth/CMakeLists.txt` and `w5k/CMakeLists.txt` — renames the WIZnet `close()` and every one of its own callers consistently, without touching the real `close()` used elsewhere |
| 8 | `main/app_main.cpp` | `struct tm` incomplete type / `localtime_r` undeclared, plus another volatile-increment deprecation | Missing `#include <time.h>` (previously arrived transitively via another header, no longer does under v6.0's cleaner header hygiene); same volatile issue as #3 on `g_mainLoopBeats` | Added `#include <time.h>`; converted `g_mainLoopBeats++` to plain assignment |

**Also ruled out as a factor**: ccache staleness. When a Kconfig change (timezone) that was
confirmed correctly saved didn't show up in a rebuilt binary, `idf.py fullclean` was used to
eliminate every possible stale-cache variable before continuing — genuinely useful step, though it
turned out the timezone issue was a separate real bug (see Part 4), not a caching problem.

---

## Part 3: Runtime issue — task watchdog resets every ~5 seconds

Once the build succeeded, the device compiled and flashed cleanly but hit a **Task Watchdog Timer
reset roughly every 5 seconds**, sustained indefinitely. This took the longest to actually
root-cause; several real leads were investigated and eliminated before finding the actual cause.

**Ruled out, in order investigated:**
1. **Direct-SPI fast path (`W5K_DIRECT_SPI`) specifically** — a backtrace initially pointed at the
   project's hand-optimized register-level SPI code (`w5k_fifo_xfer`, using `hal/spi_ll.h`
   directly). Diffed every low-level function this code touches between ESP-IDF v5.5 and v6.0.2
   (byte-for-byte identical) — not a renamed register field. A bounded 5ms timeout with
   register-state logging was added here regardless as a safety net (see Part 5), but this was not
   the actual root cause: a later backtrace showed the *normal* driver-mediated SPI path
   (`spi_device_polling_transmit()`, not the fast path) hitting the same symptom.
2. **Dynamic frequency scaling (DFS) / power management** — checked `Component config` →
   `Power Management` in menuconfig: "Support for power management" was unchecked (off). DFS isn't
   even compiled into this build. Ruled out.
2a. Also pinned `devcfg.clock_source = SPI_CLK_SRC_APB` explicitly in `W5500Eth::begin()`
    (previously left at zero-initialized default) as a hardening measure, independent of the DFS
    finding — removes any ambiguity about how the driver resolves an unset clock source.
3. **An unyielding busy-wait in the stats HTTP send path** — found and fixed a real, separate bug:
   `ntp_stats.cpp`'s `/metrics` response send loop had no yield on its retry path
   (`while (off < totalLen && esp_timer_get_time() < deadline)`, up to a 2-second deadline, no
   `vTaskDelay` when the send returns "busy"). Fixed with `vTaskDelay(1)` on the retry path. This
   **did** fix a real secondary symptom (blank `/metrics` page in a browser) but did **not** stop
   the watchdog resets, which continued afterward — proving it wasn't the root cause either,
   though it was a legitimate bug worth fixing on its own merits (a loop that can legitimately
   busy-spin for 2 full seconds is unsafe regardless).

**Actual root cause: FreeRTOS tick rate (`CONFIG_FREERTOS_HZ`).**
`ntp_wait_for_packet()` calls `ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5))`. `pdMS_TO_TICKS(5)` uses
integer math — if the tick rate is the common 100Hz default (10ms/tick), `5ms ÷ 10ms = 0` (rounds
down), and a **zero-tick timeout in FreeRTOS means "don't block at all."** This turned the intended
5ms wait into an unthrottled busy-loop: check for a notification, return instantly regardless,
call `NtpServer::loop()` (a cheap SPI check), repeat — as fast as the CPU could physically cycle,
saturating CPU1 completely and starving the idle task, which is exactly what the Task Watchdog
was catching. Every previous backtrace (SPI wait, notify-take, `NtpServer::loop()`'s opening
lines) was a snapshot of this same tight loop caught at different random instants — not
evidence of several different hangs, but one very fast spin sampled repeatedly.

This also explains why it was v6.0.2-specific: `idf.py fullclean` (used earlier to rule out
stale-cache as a factor) regenerates `sdkconfig` defaults for anything not explicitly pinned by
the project, and a tick-rate default differing between major IDF versions could silently change
this without any explicit edit — likely exactly what happened.

**Fix**: raised the FreeRTOS tick rate via `idf.py menuconfig` → `Component config` → `FreeRTOS` →
`Kernel` (tick rate setting). **Confirmed resolved** — zero watchdog triggers across a 12+ hour
run afterward.

---

## Part 4: Real bugs found, unrelated to the IDF version migration

Two genuine bugs in the project's own code, found independent of anything version-specific:

1. **Hardcoded timezone.** `components/config/config.cpp` had:
   ```cpp
   static const char* kTimezone = "AST4ADT,M3.2.0,M11.1.0";  // Atlantic time, hardcoded
   ```
   `getTimezone()` just returned this literal — **never read `CONFIG_APP_TZ` from Kconfig at all**,
   so no menuconfig edit could ever change it. Someone added the Kconfig option but never wired it
   up. Fixed: `static const char* kTimezone = CONFIG_APP_TZ;`

2. **`CONFIG_APP_USE_DISPLAY` referenced without a guard.** With the display disabled in
   menuconfig, ESP-IDF doesn't define the corresponding macro at all (no `#define CONFIG_X 0`, it's
   just absent) — but `config.cpp`'s `getUseDisplay()` referenced it directly, causing a build
   error the moment the display was turned off. Fixed with an `#ifdef` guard:
   ```cpp
   bool getUseDisplay() {
   #ifdef CONFIG_APP_USE_DISPLAY
     return CONFIG_APP_USE_DISPLAY;
   #else
     return false;
   #endif
   }
   ```

**Confirmed via direct comparison against upstream**: as of the maintainer's most recent commit
(`6bf37d1`, which happens to be the exact commit this build was already based on — nothing new to
pull), neither of these bugs nor any of the v6.0 build fixes have been addressed upstream. His
recent commits are entirely NTP-discipline/performance work (dispersion calculation tuning, IRAM
placement, 240MHz CPU, GPS metrics schema) — a different layer of the project entirely. Everything
in this document remains a genuinely needed, non-overlapping contribution.

---

## Part 5: Defensive/hardening changes actually kept

- **Bounded 5ms timeout on the direct-SPI busy-wait** (`w5k_fifo_xfer` in `w5500_eth.cpp`),
  replacing an unconditional spin. Originally added with register-state diagnostic logging
  (`cmd`, `clock`, `slave`, `user`) while this was still a suspected factor in the watchdog resets;
  once Part 3 confirmed the actual root cause was the FreeRTOS tick rate (unrelated to SPI), the
  diagnostic logging was removed as dead weight. The bounded wait itself is kept as a plain safety
  net, since an unconditional infinite spin on real hardware is inherently risky regardless of
  cause.
- **`vTaskDelay(1)`** on the busy-retry path in `ntp_stats.cpp`'s metrics send loop (see Part 3,
  item 3) — fixes a real, if secondary, failure mode.

**Reverted after root-causing the watchdog issue:** an explicit `devcfg.clock_source =
SPI_CLK_SRC_APB` pin in `W5500Eth::begin()`, originally added to rule out DFS as a factor (Part 3,
item 2). DFS was already confirmed off in menuconfig *before* this was added, so it carried no
diagnostic value and was removed as troubleshooting bloat once the real cause was found.

---

## Part 6: MQTT publishing to Home Assistant, and a real upstream `recv()` bug found along the way

Added a new component (`components/mqtt_publish/`) that publishes GPS/NTP status to an MQTT
broker with Home Assistant discovery, over the W5500's own hardware TCP socket (socket 3 — 0/1/2
are DHCP/NTP/stats). Ethernet-only, matching this deployment; WiFi was explicitly out of scope.
Because GPS/NTP correctness is non-negotiable, this is a hand-rolled, tick-driven state machine
(`MqttPublisher::loop()`, polled from `ntp_task`'s existing housekeeping slot) that does at most
one non-blocking socket call per tick and never waits on the network — it reuses only the vendored
Paho MQTT-C library's wire-format codec (`MQTTSerialize_connect`/`publish`,
`MQTTPacket_readnb`), not its synchronous `MQTTClient.c`/`mqtt_interface.c` orchestration, which
would have risked stalling NTP replies.

Getting this actually working against a real broker (EMQX) took an extended debugging session,
because the symptom — TCP connects, CONNECT is sent, then silence until a 3s timeout — had four
different real causes layered on top of each other, on top of several plausible-but-wrong network
theories that packet captures ruled out one at a time:

**Ruled out, in order investigated:** the device being on the wrong subnet from the broker (it
wasn't, once actually checked with `Test-NetConnection`); asymmetric routing/NAT between the two
subnets (pfSense confirmed doing plain routing, no NAT); pfSense/IDS actively blocking the device
after repeated connection attempts (no blocked-clients list, no IDS packages installed); and
cable-plugged-in-during-boot vs. after-boot timing (no effect on the actual symptom — the
connection was already long-established by the time it mattered).

**Bug 1 — reused source port across reconnects.** `w5k_tcp_connect()` (added to
`components/w5k/w5k_tcp_wrapper.c` alongside the existing `w5k_tcp_listen()`) originally picked a
fixed ephemeral port (`20000 + socket_num`) every time. A packet capture showed the exact
signature of a stale connection-tracking entry colliding with this: several fully-silent SYN
cycles in a row (no SYN-ACK at all), then one that got through instantly, repeating. Fixed by
varying the source port on every call (`20000 + socket_num + counter`).

**Bug 2 — CONNACK discarded on immediate broker close.** `mqttNonBlockingRead()` (the
`MQTTPacket_readnb` transport callback) originally checked the socket's connection status *before*
attempting to read. A broker rejecting a connection commonly sends its CONNACK and then
immediately closes — by the time the check ran, the socket already showed `CLOSE_WAIT`, so the
CONNACK sitting right behind that transition in the RX buffer was never read, and a real rejection
was misreported as a bare "connection closed." Fixed by reading first, and treating `CLOSE_WAIT`
as "keep trying" rather than a hard error.

**Bug 3 — auth field truncation.** `username`/`password`/`clientId` were originally fixed
32-byte `char[]` members; a real password happened to be exactly 32 characters, silently truncated
to 31 by `snprintf`, producing a broker-side "bad username/password" (CONNACK rc=4) with no
firmware-side indication anything was wrong. Fixed by heap-allocating these three fields to fit
exactly what's configured (`dupString()`), matching MQTT 3.1.1's real 65535-byte limit instead of
an arbitrary cap.

**Bug 4 — the actual root cause, and the one that took the longest to find.** Even with 1-3 fixed,
a packet capture proved the broker was sending a valid 4-byte CONNACK within a few milliseconds of
every single CONNECT, yet the firmware polled for a full 3 seconds and never once saw it —
`rxAnyByteSeen`/poll-count instrumentation added specifically to chase this showed the read
callback was being called at the expected ~20ms cadence (142 times over 3s) but never returning a
single byte, despite the socket genuinely showing `ESTABLISHED` throughout. Root cause: `recv()`
in the vendored `ioLibrary_Driver/Ethernet/socket.c` checks whether the socket is in non-blocking
mode *before* checking whether data has actually arrived:
```c
if (sock_io_mode & (1 << sn)) {
    return SOCK_BUSY;      // fires unconditionally, even with data already waiting
}
if (recvsize != 0) {
    break;
}
```
For a non-blocking socket this returns `SOCK_BUSY` on literally every call, regardless of how much
data is sitting in the RX buffer — the `recvsize` check is unreachable in non-blocking mode. This
had never surfaced before because nothing else in the project calls `recv()` on a non-blocking TCP
socket: the stats HTTP server's socket stays in blocking mode, and NTP is UDP with entirely
separate `sendto`/`recvfrom` fast-path code. MQTT is the first thing here to combine a
non-blocking TCP socket with this exact function — and needed non-blocking mode specifically so
`w5k_tcp_connect()`'s `connect()` call wouldn't spin the shared `ntp_task` (see Bug 1's context;
non-blocking mode is set via `ctlsocket(..., SOCK_IO_NONBLOCK, ...)` right after `socket()`,
before `connect()`).

The confirming detail: the same function's `#ifdef IPV6_AVAILABLE` branch, three lines above the
broken one, has the two checks in the *correct* order (data-available wins over would-block) —
this was a copy-paste divergence in the vendored library, not an intentional design choice, and
the fix is exactly swapping the two `if` blocks in the non-`IPV6_AVAILABLE` branch to match.

Once this landed, MQTT connected on the very first attempt and stayed connected — confirmed
against a real EMQX broker with Home Assistant showing the device and all 14 entities populated
under its MQTT integration. The `rxAnyByteSeen`/poll-count diagnostics used to chase Bug 4, and a
speculative (and, once Bug 4 was found, unnecessary) workaround for a suspected `MQTTPacket_readnb`
bitfield-decode ambiguity, were both removed afterward as troubleshooting bloat — same practice as
Part 5.

**Follow-up: display precision.** Once connected, Home Assistant showed `Clock Offset`/`PPS Jitter`
rounded down to a misleading `0.00000` — HA's per-entity display precision (both the manual picker
and the `suggested_display_precision` discovery hint) caps out at 7 decimal places, but
`offset_s`/`pps_jitter_s` are published with `%.9f` (nanosecond resolution, the whole point of this
hardware) and `root_dispersion_s` with `%.6f`. Rather than fight that ceiling, switched all three to
**microseconds** instead of seconds (`offset_us`/`pps_jitter_us`/`root_dispersion_us`, `%.3f` —
still 1ns resolution, just expressed as a small ordinary-looking number like `0.007` instead of
`0.000000007`) with `suggested_display_precision: 3`. This was a payload schema change: the old
`_s`-suffixed entities went stale in HA once the device republished discovery with the new keys,
and needed a manual cleanup pass (`Settings > Devices & Services > Entities`, filter to
unavailable, delete the three old ones). **Confirmed working** after flashing: HA shows real
decimal values (`Clock Offset -1.319 µs`, `PPS Jitter 0.007000 µs`, `Root Dispersion 17.46 µs`)
instead of the earlier `0.00000` rounding.

**Someone else found this independently.** [`Wiznet/ioLibrary_Driver#183`](https://github.com/Wiznet/ioLibrary_Driver/pull/183)
("fix: check recvsize before nonblocking mode in TCP recv() (AUD-004)"), opened by
`occamsshavingkit` on 2026-07-18 — about three weeks before we hit it here — proposes the exact
same reorder at the exact same lines. It was still unmerged as of 2026-08-08, which is why the
submodule points at a fork (`github.com/Bestivus/ioLibrary_Driver`, branch `esp32-ntp-fixes`)
rather than waiting on it. **If/when #183 merges upstream**, `.gitmodules` can be pointed back at
`Wiznet/ioLibrary_Driver` directly and the fork dropped — no need to file a duplicate report.

---

## Files modified from upstream, complete list

- `CMakeLists.txt` (top-level) — missing-field-initializer warning suppression
- `components/w5500_eth/CMakeLists.txt` — added `esp_driver_gpio` requirement; added scoped
  `close()` rename for `socket.c`/`dhcp.c`
- `components/w5500_eth/w5500_eth.cpp` — 5x volatile-increment fixes; bounded SPI timeout as a
  plain safety net (no diagnostic logging, no clock-source pin — both removed once root-caused)
- `components/gps/CMakeLists.txt` — added `esp_driver_gpio` and `esp_driver_uart` requirements
- `components/w5k/CMakeLists.txt` — added scoped `close()` rename for the two wrapper `.c` files
- `components/ntp_stats/ntp_stats.cpp` — `SOCK_STREAM`/`SOCK_DGRAM` push/pop macro guard;
  `vTaskDelay(1)` on busy-retry path
- `components/config/config.cpp` — timezone now reads `CONFIG_APP_TZ`; `getUseDisplay()` `#ifdef`
  guard
- `main/app_main.cpp` — added `#include <time.h>`; 1x volatile-increment fix; wires up
  `MqttPublisher` (Part 6)
- `sdkconfig` — FreeRTOS tick rate raised from the post-`fullclean` default
- `ioLibrary_Driver/Ethernet/socket.c` — fixed `recv()`'s non-blocking-mode check firing before its
  data-available check, so a non-blocking TCP socket could never actually read data (Part 6, Bug 4)
- `components/w5k/w5k_tcp_wrapper.c`/`.h` — added `w5k_tcp_connect()`, the client-mode counterpart
  to the existing `w5k_tcp_listen()`, with a per-call ephemeral source port (Part 6, Bug 1)
- `main/Kconfig.projbuild`, `components/config/config.h`/`.cpp` — new `APP_MQTT_*` options and
  accessors (Part 6)
- `components/mqtt_publish/` — new component, not an upstream file (Part 6)
