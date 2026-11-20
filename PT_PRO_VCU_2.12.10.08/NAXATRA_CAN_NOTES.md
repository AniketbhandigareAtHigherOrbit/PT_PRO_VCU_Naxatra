# Naxatra CAN integration

All three motor paths use standard classic CAN frames at 500 kbit/s.
Keep the controllers on separate physical buses: left CAN2, right CAN0, rotor CAN1.
The common IDs cannot address multiple Naxatra controllers independently on one bus.

Commands: ID 0x415, DLC 8, dedicated 50 ms scheduler task. Drive logic remains 20 ms.
Byte 0 zero, byte 1 gear/reverse/CAN/brake, byte 2 0x01 (soft speed mode),
bytes 3-4 big-endian target RPM, bytes 5-6 acceleration/deceleration 0x14,
byte 7 zero. Ramps represent 2.0 seconds provisionally.

IMPORTANT: The supplied v05 DBC overlaps RPM and acceleration in byte 4.
The implementation uses the log-supported correction: acceleration byte 5,
deceleration byte 6. Both ramp values were identical in working log commands;
individual ramp meanings still need confirmation from Naxatra. The source DBC
has not been edited. Runtime limits remain 1430 RPM for wheels, 3000 RPM for rotor.
No fixed 40-percent speed has been introduced.

Each motor sends zero speed for 500 ms after a valid enable: 0x28 forward or
0x38 reverse, brake off. This repeats after a stop, including a late lever press.
Kill switch, disabled commands,
invalid direction, or command-cache age >=100 ms send zero speed, Park and brake.
The physical controller response to Park/brake and its command-loss timeout
must be verified on the bench. No hardware flashing was performed.
Legacy 2 RPM braking on kill is removed. Active-brake and slope handover are
disabled. Direction selection remains on the existing E2 GPIO input (0 forward,
1 reverse); no new dashboard CAN direction message is introduced. Direction
changes retain zero-speed feedback supervision before applying the new direction.
Physical startup and reversal behavior still require bench validation.

Feedback: standard 0x411 and 0x412, attributed by CAN channel. RPM is big-endian;
temperatures are signed degrees C using raw-40; bus current is raw-10000 in 0.1 A;
voltage is converted from V into existing decivolt units. Existing motor_current_A
now represents phase RMS current. Precise bus current is retained separately.
Fault word low byte: status2 & 0xFE; high byte: status3 & 0xF0.
Electric/rate-limit state bits do not become fault codes.
Actual direction comes from status1 bit4; the protocol has no independent command
direction echo, so command_dir remains DIR_STOP/unknown. Direction supervision
uses fresh 0x411 sequences, while existing time-based acknowledgement fallback
is preserved. Telemetry temperature bytes retain their existing 0..255 clamping.

Validation and platform limitations are recorded in the chat change report.
