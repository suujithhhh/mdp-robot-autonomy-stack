# CAN layer (simulated CAN 2.0A)

`src/can_bus` models a classical CAN bus in C++17 and maps the robot's UART protocol onto it, so
the whole mission (planner -> executor -> driver -> motor controller) can run over CAN instead of a
UART. It is a **simulation**: nothing here has run on a real CAN transceiver or an STM32 bxCAN
peripheral.

## What CAN is, in one paragraph

A two-wire differential multi-drop bus. Every message is a *frame*: an 11-bit identifier plus up
to 8 data bytes. Nodes broadcast by identifier (no addresses), and every node hears every frame.
If two nodes start at once, **bitwise arbitration** lets the lower identifier win without
corrupting either frame. Frames carry a CRC-15, are bit-stuffed so the line keeps toggling, and
are acknowledged by any receiver; failures are retransmitted automatically and tracked by error
counters that can switch a misbehaving node off the bus.

## What is implemented

| Piece | Where | Detail |
|---|---|---|
| Frame encoding | `frame.cpp` | Bit-exact standard data frame: SOF, 11-bit ID, RTR/IDE/r0, DLC, data, CRC-15, delimiters, ACK slot, EOF, intermission |
| Bit stuffing | `frame.cpp` | A complementary bit after every 5 equal bits (stuff bits count toward the next run); six equal bits = stuff error |
| CRC-15/CAN | `frame.cpp` | Polynomial 0x4599. Checked against the published catalogue value for "123456789" (0x059E) |
| Decoding with checks | `frame.cpp` | Stuff, CRC, form (truncation, bad SOF/delimiter), DLC > 8, unsupported (remote frames) |
| Bus + arbitration | `bus.cpp` | Wired-AND bit by bit over SOF + ID; a node that sends 1 while the bus reads 0 loses and retries |
| Bit errors | `bus.cpp` | Random per-bit corruption of the transmitted stream; receivers detect it through the checks above |
| ACK slot | `bus.cpp` | A frame no node acknowledges is an ACK error and is retransmitted |
| Error counters | `bus.cpp` | TEC +8 on a transmit error, -1 on success; error-passive above 127; bus-off above 255 until `reset_bus_off()` |
| Timing | `bus.cpp` | Each frame costs its real stuffed bit count at the configured bit rate (default 500 kbit/s) |
| Robot message map | `robot_can.cpp` | See below; two-frame messages are rebuilt by `HostDecoder` |

## Message map

Lower identifier = higher priority, so STOP outranks everything and telemetry yields to commands.

| ID | Direction | Meaning | Data (little endian) |
|---|---|---|---|
| `0x001` | host -> device | STOP | none |
| `0x010` | host -> device | MOVE | cmd u8, value u16 |
| `0x011` | host -> device | IMU_GET | none |
| `0x080` | device -> host | REPLY | type u8 (ACK / ACK_STOPPED / ERR), error code u8 |
| `0x081` | device -> host | IMU_READING | centi-degrees i16 |
| `0x0F0` | device -> host | EVENT_START | cmd u8 |
| `0x0F1` / `0x0F2` | device -> host | EVENT_END (2 frames) | cmd, result, duration ms / DL, DR counts i32 |
| `0x100` / `0x101` | device -> host | TELEMETRY (2 frames) | left, right counts i32 / t ms u32, heading centi-deg i16 |

The driver (`BridgeCore`) and the device model (`VirtualStm`) are unchanged: `CanSimLink` converts
their text lines to and from frames at the edge, which is also a check that the CAN mapping
preserves every message.

## Measured (simulation, `mdp_sim_mission --can <bit error rate>`)

3-obstacle mission, 9 commands, 500 kbit/s:

| Run | Frames delivered | Corrupted + retransmitted | Bus utilisation | Final pose vs UART run |
|---|---|---|---|---|
| Clean bus | 345 | 0 | 0.50 % | identical |
| Bit error rate 5e-4 | 345 | 8 | 0.51 % | identical |

Both nodes finish error-active with TEC 0.

### Robustness sweep (100 seeded missions per row, same 3-obstacle mission)

| Link | Injected fault | Missions completed | Corrupted frames retransmitted (total) |
|---|---|---|---|
| UART | 0.2 % of device bytes lost | 92 / 100 | n/a (lost bytes are not recovered) |
| UART | 0.5 % of device bytes lost | 85 / 100 | n/a |
| UART | 2 % of device bytes lost | 39 / 100 | n/a |
| CAN | bit error rate 5e-4 (about 5 % of frames corrupted) | **100 / 100** | 1652 |
| CAN | bit error rate 2e-3 (about 16 % of frames) | 55 / 100 | 6539 |
| CAN | bit error rate 5e-3 | 0 / 100 | 4171 |

How to read this honestly:

* The two fault models are not the same unit (a lost byte vs a flipped bit), so the rows are not a
  like-for-like contest; they show the *shape* of each failure mode.
* CAN detects every corrupted frame and retransmits it, so occasional corruption costs only a
  little bus time and never changes the outcome (100 / 100, identical final pose).
* Under a **persistently** noisy bus the transmit error counter keeps climbing (+8 per error, -1
  per success) and the node goes **bus-off**: it falls silent instead of flooding a bad bus. The
  mission then aborts with a link timeout. That is the intended fail-silent behaviour of CAN, and it
  is why a real system still needs a plan for a bus-off motor controller.
* The UART driver never resends a command whose outcome is unknown (no command IDs), so a single
  lost byte in a command or reply ends in a link timeout; CAN's frame-level acknowledgement and
  retransmission removes that failure mode for transient corruption only.

## Tests (72)

* `can_bus_tests` (59): CRC (catalogue value, zero remainder property, every single-bit flip
  changes the CRC), stuffing (worked examples, inverse property, no six-bit runs), frame round
  trips for every DLC and extreme IDs, **every single-bit error in the stuffed region detected**,
  truncation / bad SOF / DLC / remote-frame handling, arbitration (lowest ID wins, three
  contenders, IDs differing in the last bit, STOP overtakes queued telemetry, per-node FIFO),
  timing equals the bit count, queue overflow, retransmission after bit errors (50 frames, none
  lost or duplicated), ACK errors, error-passive then bus-off, bus-off recovery, disabled nodes,
  duplicate identifiers, and the message map (round trips of every command, reply, event and
  telemetry message, two-frame pairing, malformed frames).
* `mdp_mission_tests` (+13): the full mission over CAN is identical to the UART run, odometry,
  goal tolerance, bus load < 5 %, error counters, bit-error recovery over 10 seeds, heavy
  errors (completes or ends with a definite outcome in bounded time), a dead motor controller (host goes bus-off, mission aborts with a link timeout), STOP
  latency under load, stalled motors, IMU failure, wheel slip.

## Limits (deliberately not modelled)

Remote frames, 29-bit extended identifiers, CAN FD, the physical layer and propagation delay,
bit-level resynchronisation, the error-passive suspend time, automatic bus-off recovery, hardware
acceptance filters, and multiple transmit mailboxes (each node sends its queue first-in first-out;
a real controller would pick its lowest identifier). Real hardware (an STM32 in bxCAN loopback
mode) is the natural next step.
