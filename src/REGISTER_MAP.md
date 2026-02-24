# Runtime Configuration Reference

## How It Works

Everything in the system — effect parameters, signal routing, bypass switches — is controlled through a single flat register space. You write to it from a serial terminal (115200 8N1) using text commands. Each command becomes a `wr_en/wr_addr/wr_data` pulse that travels through CDC into the audio domain and hits `ctrl_bus`, which routes the write to the correct destination.

```
Terminal (UART) → cmd_proc_v2 → wr_en/wr_addr/wr_data → CDC → ctrl_bus
                                                                  │
                                              ┌───────────────────┼───────────────┐
                                              ▼                   ▼               ▼
                                        slot regs[0..7]     route[0..5]     bypass[0..3]
                                        (per-effect knobs)  (crossbar)      (on/off)
```

---

## Complete Address Map

### Slot 0 — Tremolo (0x00..0x07)

| Addr | Name      | Bits | Core Port    | Description                    |
|------|-----------|------|--------------|--------------------------------|
| 0x00 | rate      | 7:0  | rate_val     | LFO speed (0=stop, 255=fast)   |
| 0x01 | depth     | 7:0  | depth_val    | Mod depth (0=unity, 255=full)  |
| 0x02 | shape     | 0    | shape_sel    | 0=triangle, 1=square           |
| 0x03–0x07 | —   |      |              | Reserved                       |

### Slot 1 — Phaser (0x08..0x0F)

| Addr | Name      | Bits | Core Port    | Description                    |
|------|-----------|------|--------------|--------------------------------|
| 0x08 | speed     | 7:0  | speed_val    | Sweep speed                    |
| 0x09 | fben      | 0    | feedback_en  | 0=off, 1=feedback on           |
| 0x0A–0x0F | —   |      |              | Reserved                       |

### Slot 2 — Chorus (0x10..0x17)

| Addr | Name      | Bits | Core Port    | Description                    |
|------|-----------|------|--------------|--------------------------------|
| 0x10 | rate      | 7:0  | rate         | LFO rate                       |
| 0x11 | depth     | 7:0  | depth        | Modulation depth               |
| 0x12 | efx       | 7:0  | effect_lvl   | Wet/dry mix (0=dry, 255=wet)   |
| 0x13 | eqhi      | 7:0  | e_q_hi       | High EQ                        |
| 0x14 | eqlo      | 7:0  | e_q_lo       | Low EQ                         |
| 0x15–0x17 | —   |      |              | Reserved                       |

### Slot 3 — DD3 Delay (0x18..0x1F)

| Addr | Name      | Bits | Core Port    | Description                    |
|------|-----------|------|--------------|--------------------------------|
| 0x18 | tone      | 7:0  | tone_val     | Tone filter (0=dark, FF=bright)|
| 0x19 | level     | 7:0  | level_val    | Wet level (0=off, FF=full)     |
| 0x1A | feedback  | 7:0  | feedback_val | Feedback amount (0=off)        |
| 0x1B | time[7:0] | 7:0  | time_val     | Delay time, byte 0 (LE)        |
| 0x1C | time[15:8]| 7:0  | time_val     | Delay time, byte 1             |
| 0x1D | time[23:16]| 7:0 | time_val     | Delay time, byte 2             |
| 0x1E | time[31:24]| 7:0 | time_val     | Delay time, byte 3             |
| 0x1F | —         |      |              | Reserved                       |

The four time bytes are assembled inside the slot as: `time_val_32 = {regs[6], regs[5], regs[4], regs[3]}`, which maps to `{0x1E, 0x1D, 0x1C, 0x1B}`. The `set dly time` command handles the 4-byte split automatically.

### Crossbar Routes (0x40..0x45)

| Addr | Name     | Bits  | Description                        |
|------|----------|-------|------------------------------------|
| 0x40 | route[0] | 2:0   | Source for slot 0 input            |
| 0x41 | route[1] | 2:0   | Source for slot 1 input            |
| 0x42 | route[2] | 2:0   | Source for slot 2 input            |
| 0x43 | route[3] | 2:0   | Source for slot 3 input            |
| 0x44 | route[4] | 2:0   | (unused port)                      |
| 0x45 | route[5] | 2:0   | Source for DAC input               |

Route values select which **master port** feeds the given **slave port**:

| Value | Master Port            |
|-------|------------------------|
| 0     | ADC (I2S input)        |
| 1     | Slot 0 output (tremolo)|
| 2     | Slot 1 output (phaser) |
| 3     | Slot 2 output (chorus) |
| 4     | Slot 3 output (delay)  |

Reset default: `route[k] = k` (linear chain).

### Bypass Flags (0x48..0x4B)

| Addr | Name      | Bits | Description                         |
|------|-----------|------|-------------------------------------|
| 0x48 | bypass[0] | 0    | 1 = bypass slot 0 (tremolo)         |
| 0x49 | bypass[1] | 0    | 1 = bypass slot 1 (phaser)          |
| 0x4A | bypass[2] | 0    | 1 = bypass slot 2 (chorus)          |
| 0x4B | bypass[3] | 0    | 1 = bypass slot 3 (delay)           |

Bypass is OR'd with the hardware switches: `bypass_merged[i] = ctrl_bypass[i] | ~sw_effect[i]`. Either the register OR the physical switch can bypass an effect.

---

## CLI Command Reference

### Effect Parameters

```
set trm rate <hex>          Tremolo LFO rate
set trm depth <hex>         Tremolo modulation depth
set trm shape <hex>         Tremolo waveform (0=tri, 1=sq)

set pha speed <hex>         Phaser sweep speed
set pha fben <hex>          Phaser feedback (0=off, 1=on)

set cho rate <hex>          Chorus LFO rate
set cho depth <hex>         Chorus mod depth
set cho efx <hex>           Chorus wet/dry level
set cho eqhi <hex>          Chorus high EQ
set cho eqlo <hex>          Chorus low EQ

set dly tone <hex>          Delay tone filter
set dly level <hex>         Delay wet level
set dly feedback <hex>      Delay feedback amount
set dly time <hex>          Delay time (up to 8 hex digits, 32-bit)
```

### Routing

```
set route <port> <source>   Set crossbar route
set bypass <slot> <val>     Set bypass flag (0/1)
```

### Raw Write

```
wr <addr><data>             Write any register (hex, no space: "wr 4003")
```

### Status

```
st                          Print current register values
```

All values are hexadecimal. No `0x` prefix needed.

---

## Routing Examples — Full Terminal Sessions

### Example 1: Default Linear Chain

This is what you get at power-on. No commands needed.

```
>> st
TRM 3C B4 00
PHA 50 00
>> 
```

Signal path: `ADC → tremolo → phaser → chorus → delay → DAC`

Equivalent explicit configuration:
```
>> set route 0 0
OK
>> set route 1 1
OK
>> set route 2 2
OK
>> set route 3 3
OK
>> set route 5 4
OK
```

### Example 2: Reorder Effects (Delay First, Then Chorus)

Goal: `ADC → delay → chorus → phaser → tremolo → DAC`

```
>> set route 3 0
OK
>> set route 2 4
OK
>> set route 1 3
OK
>> set route 0 2
OK
>> set route 5 1
OK
```

What this means:
- `route[3]=0` → slot 3 (delay) gets its input from master 0 (ADC)
- `route[2]=4` → slot 2 (chorus) gets its input from master 4 (delay output)
- `route[1]=3` → slot 1 (phaser) gets its input from master 3 (chorus output)
- `route[0]=2` → slot 0 (tremolo) gets its input from master 2 (phaser output)
- `route[5]=1` → DAC gets its input from master 1 (tremolo output)

### Example 3: Single Effect Only (Delay)

Goal: `ADC → delay → DAC` — skip everything else.

```
>> set route 3 0
OK
>> set route 5 4
OK
>> set bypass 0 1
OK
>> set bypass 1 1
OK
>> set bypass 2 1
OK
```

Slots 0/1/2 are bypassed and their routing doesn't matter. Only slot 3 and the DAC route matter.

### Example 4: Full Bypass (Clean Signal)

Goal: `ADC → DAC` — no effects at all.

```
>> set route 5 0
OK
```

One command. DAC now reads directly from ADC, bypassing all slots entirely. The effects still receive data from wherever their routes point, but nothing reaches the DAC except the raw ADC signal.

### Example 5: Parallel Fan-Out (Tremolo + Phaser Both From ADC)

Goal: Split the ADC to two effects simultaneously.

```
>> set route 0 0
OK
>> set route 1 0
OK
>> set route 5 1
OK
```

Both slot 0 (tremolo) and slot 1 (phaser) now read from the ADC. The DAC in this example listens to tremolo's output. The phaser output is available on master 2 but isn't routed to the DAC.

For a true mix of both, you'd need a mixer slot (future extension). But you can A/B between them instantly:

```
>> set route 5 1
OK
```
(DAC ← tremolo)

```
>> set route 5 2
OK
```
(DAC ← phaser)

### Example 6: Configure Effect Parameters and Routing Together

Goal: Set up a slapback delay (short time, no feedback) into chorus, skip tremolo and phaser.

```
>> set dly level 80
OK
>> set dly feedback 00
OK
>> set dly time 000005DC
OK
>> set dly tone FF
OK
>> set cho rate 50
OK
>> set cho depth 60
OK
>> set cho efx 80
OK
>> set route 3 0
OK
>> set route 2 4
OK
>> set route 5 3
OK
>> set bypass 0 1
OK
>> set bypass 1 1
OK
```

Signal path: `ADC → delay(slapback) → chorus → DAC`

Delay time = 0x5DC = 1500 samples ≈ 31 ms at 48 kHz.

### Example 7: Hot-Swap Routing Without Glitch

The crossbar is purely combinational, so route changes take effect on the very next clock edge. At 48 kHz sample rate, you have ~256 audio clocks between samples. A UART command takes ~1ms to process, which spans ~50 sample periods. So the route change happens cleanly between samples — no partial samples, no glitches.

```
>> set route 5 4
OK
```
(now hearing delay output)

```
>> set route 5 0
OK
```
(now hearing clean ADC — instant switch)

### Example 8: Raw Register Write (Power User)

If you know the address, skip the named commands:

```
>> wr 1980
OK
```

This writes `0x80` to address `0x19`, which is `dly level`. Same as `set dly level 80`.

```
>> wr 4000
OK
```

This writes `0x00` to address `0x40`, which is `route[0]`. Slot 0 now reads from ADC.

---

## Data Flow Summary

```
                    ┌──────────────────────────────────────────────────┐
   UART ──→ cmd ──→ │ wr_en/wr_addr/wr_data ──→ CDC ──→ ctrl_bus     │
                    │                                      │  │  │    │
                    │    ┌─── slot regs ◄──────────────────┘  │  │    │
                    │    │    route[] ◄────────────────────────┘  │    │
                    │    │    bypass[] ◄──────────────────────────┘    │
                    │    ▼                                             │
                    │  ┌──────────────────────────────────────────┐    │
                    │  │ axis_crossbar (route[] selects paths)   │    │
   ADC ─→ m[0] ────┤  │  s[0] → slot0(trem) → m[1]             │    │
                    │  │  s[1] → slot1(pha)  → m[2]             │    │
                    │  │  s[2] → slot2(cho)  → m[3]             │    │
                    │  │  s[3] → slot3(dly)  → m[4]             │    │
   DAC ←─ s[5] ←───┤  │                                        │    │
                    │  └──────────────────────────────────────────┘    │
                    └──────────────────────────────────────────────────┘
```
