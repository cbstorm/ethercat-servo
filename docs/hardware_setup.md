# Hardware setup and commissioning

> Commission with the motor **decoupled from the mechanics** and a **hard-wired E-stop / STO** within reach. Never test motion unattended. The examples and the MQTT example refuse to command motion on the hardware backend unless you pass `--allow-motion`.

## What the supplied documents actually describe

| File | What it is | Used for |
|---|---|---|
| `LC-EH_V1.5.xml` | EtherCAT ESI of vendor 0x0A79 (Shenzhen Xinlichuan). It has 10 devices, including **ISL-EC** (product 0x5800, rev 0x0001). | PDO defaults, InitCmds, DC settings and object flags. Copied to `esi/`. |
| `ISL60-PR用户使用手册26-01-21 en-GB.pdf` | Manual of the **ISL60-PR**. This variant is controlled by **pulse/direction and RS485 Modbus**; the manual says the series also has CANopen and EtherCAT variants. | Background only: encoder resolution (17/23 bit), default electronic gear 131072/10000, homing method descriptions, error codes (`Er.xxx`). |
| `ISL_C_V1.0.eds` | CANopen EDS of the ISL CANopen variant | **Not** used as an ESI. The ESI parser rejects it. |

**An ISL60-PR drive cannot be controlled over EtherCAT.** This library needs the EtherCAT variant (ISL-EC / ISL60-EC or similar). Confirm the exact model on the nameplate and obtain the **EtherCAT manual of that variant**. The PR manual's parameter groups (P0x-xx) may not match the EtherCAT firmware's object dictionary.

## Findings from the ISL-EC ESI (verify on the real drive)

1. **Default PDO assignment** (0x1C12 = {0x1600}, 0x1C13 = {0x1A00}):

   - RxPDO 0x1600: 0x6040 Controlword, 0x607A Target position, 0x60FF Target velocity, 0x60B8 Touch probe function, 0x60FE:01 Digital outputs.
   - TxPDO 0x1A00: 0x603F, 0x6041, 0x6064, 0x60FD, 0x606C, 0x60F4, 0x60B9, 0x60BA..0x60BD, 0x6061.

   **0x6060 Modes of operation is not in the default RxPDO.** It sits in 0x1601, which is not assigned. The ESI sets the mode to 8 (CSP) with an InitCmd at PREOP→SAFEOP. This differs from a mapping that contains 0x6060 + 0x607A only.
2. **0x6071 Target torque and 0x6077 Torque actual are mappable but not mapped by default.** `config/example.yaml` remaps 0x1600/0x1A00 to include 0x6060, 0x6071, 0x6077 and 0x6061. The ESI declares `PdoAssign`/`PdoConfig` = true, but the firmware must still accept the remapping.
3. **0x6502 Supported drive modes = 0x1DD**, decoded per CiA 402: PP, PV, profile torque, IP, CSP, CSV. **Not set: Homing (bit 5) and CST (bit 9)**, while reserved bit 4 is set. The ESI also declares the homing objects 0x6098/0x6099/0x609A, so bit 4 may be a vendor encoding error for homing. The library warns about this at configuration time. At start-up it **reads 0x6502 from the drive** and refuses to start if a configured mode is missing (`safety.verify_supported_modes`).
4. DC: `AssignActivate` 0x0300 (SYNC0). An SM-synchronous mode also exists (AssignActivate 0).
5. Sync managers: SM2 outputs at 0x1200 (max 68 bytes), SM3 inputs at 0x1300 (max 68 bytes). The mapping in `example.yaml` uses 13 bytes out and 21 bytes in.

## Commissioning procedure

1. **Identify the bus.**

   ```bash
   ethercat slaves -v                                     # IgH tool
   ethercat_motion_scan --esi esi/LC-EH_V1.5.xml          # identity, ESI match, 0x6502 read from the drive
   ```

   Write the actual vendor ID, product code and revision into the config. Keep `revision` set so a different firmware fails at start-up instead of running.
2. **Read the object dictionary of the real drive** (PREOP):

   ```bash
   ethercat upload -p0 0x6502 0      # supported modes
   ethercat upload -p0 0x1C12 0      # PDO assignment
   ethercat upload -p0 0x608F 1      # position encoder resolution (if implemented)
   ethercat upload -p0 0x6091 1 ; ethercat upload -p0 0x6091 2   # gear ratio
   ethercat upload -p0 0x605A 0      # quick stop option code
   ```
3. **Decide the PDO mapping.** Start with `pdo: {source: esi}` (vendor default, mode switching by SDO). Move to the explicit mapping in `example.yaml` only after confirming the drive accepts it: the master log shows PDO configuration errors otherwise.
4. **Units.** Set `counts_per_revolution` to the drive's position units per motor revolution after its own electronic gear (the ISL PR manual's default gear 131072/10000 gives 10000 units/rev; the EtherCAT firmware uses 0x6091/0x608F). Measure it: command a small move and compare with the mechanics. Determine the units of 0x606C/0x60FF (`velocity_factor`) and 0x6083/0x6084 (`acceleration_factor`) from the drive manual.
5. **Check the configuration offline:** `ethercat_motion_diagnostic config.yaml --check`.
6. **Bring the bus up without enabling:** `ethercat_motion_diagnostic config.yaml --seconds 30`. Expected: WC complete, all slaves OP, drive in `SwitchOnDisabled`, correct mode display, plausible position, no overruns.
7. **Set the drive-side protections** using the vendor tool or SDO: torque limit (0x6072), following error window (0x6065), quick stop deceleration (0x6085) and option code (0x605A), and the SM watchdog / 0x10F1 error reaction on communication loss.
8. **First enable with the motor decoupled:** `basic_axis config.yaml --distance 1 --allow-motion`. Check the direction (`direction`), the scaling, and that stop / quick stop / E-stop behave as expected.
9. **Tune the limits.** `following_error_max` must exceed the real lag at maximum speed. Set the soft limits inside the mechanical range.
10. Only then couple the mechanics and increase speed step by step.

## Hardware verification checklist

| # | Item | How | Done |
|---|---|---|---|
| 1 | Drive is an EtherCAT variant (not ISL60-PR) | Nameplate, `ethercat slaves` | ☐ |
| 2 | Vendor / product / revision match the config | `ethercat_motion_scan` | ☐ |
| 3 | Firmware's actual 0x6502 contains every configured mode | Start-up check / `ethercat upload 0x6502` | ☐ |
| 4 | Homing supported, and which 0x6098 methods | Drive EtherCAT manual, test | ☐ |
| 5 | PDO remapping accepted (if not using the ESI default) | Master log, `ethercat pdos` | ☐ |
| 6 | Position units per revolution (`counts_per_revolution`) | Measured move | ☐ |
| 7 | Velocity / acceleration units (`velocity_factor`, `acceleration_factor`) | Manual + PV/PP test | ☐ |
| 8 | Direction of rotation (`direction`) | First move | ☐ |
| 9 | DC SYNC0 works at the chosen cycle time (1 ms) and shift | `ethercat slaves -v`, 0x1C32 sync errors | ☐ |
| 10 | Drive reaction to communication loss (SM watchdog, 0x10F1, 0x6007) | Unplug the cable while enabled, motor decoupled | ☐ |
| 11 | Quick stop option code (0x605A) and deceleration (0x6085) | Quick stop test | ☐ |
| 12 | Fault reset behavior and error codes (0x603F) | Provoke a fault | ☐ |
| 13 | STO / hard-wired E-stop independent of software | Electrical review | ☐ |
| 14 | Following-error and overspeed thresholds | Runs at max speed | ☐ |
| 15 | Soft limits inside the mechanical travel; homing reference correct | Measurement | ☐ |
| 16 | Real-time timing on the target PC (jitter, overruns) | `ethercat_motion_diagnostic --seconds 600` | ☐ |
| 17 | Brake control (if the motor has a holding brake) | Drive manual, test with load | ☐ |

## Unresolved hardware questions

- The exact drive model and firmware in use, and whether an EtherCAT-specific manual is available.
- Whether 0x6502 bit 4 really means homing on this firmware, and which homing methods are implemented.
- Whether the firmware accepts mapping 0x6060/0x6071/0x6077 into 0x1600/0x1A00, or requires 0x1601 for 0x6060.
- The units of 0x606C/0x60FF/0x6081/0x6083 (some drives use rpm or 0.1 rpm).
- Whether 0x6077 torque is in 0.1 % of rated torque (CiA 402 default, assumed here).
- The SM watchdog timeout, and the drive's reaction when the master stops sending.
