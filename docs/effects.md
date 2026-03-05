# GTFX Effects Reference

Developer and hardware reference for the GTFX FPGA guitar effects processor.

---

## 1. System Overview

| Item | Detail |
|------|--------|
| Target | Arty A7-100T (xc7a100tcsg324-1) |
| Audio format | 24-bit signed, 48 kHz stereo |
| AXI-Stream tdata | 48-bit: `{left[23:0], right[23:0]}` |
| Effect slots | 10 (slots 0-9, each wrapped by `axis_effect_slot`) |
| Routing | 11-port symmetric AXI-Stream crossbar |
| Control | UART CLI at 115200 baud |

### Clock Domains

| Domain | Frequency | Purpose |
|--------|-----------|---------|
| `sys_clk` | 100 MHz | UART, command processing |
| `clk_audio` | 12.288 MHz (Clock Wizard) | Effects DSP, I2S, crossbar |

CDC between domains uses a toggle handshake with 2-flop synchronizer in `top.sv`. The `cmd_proc` enforces a 31-cycle wait between writes to guarantee capture.

### Audio Helpers (`axis_audio_pkg.sv`)

```
pack_stereo(left, right)  → {left, right}   (48-bit)
unpack_left(tdata)        → tdata[47:24]
unpack_right(tdata)       → tdata[23:0]
saturate(val)             → clip 25-bit to ±8388607
```

### Default Signal Chain

```
ADC(0) → CMP(9) → BMF(7) → TUB(5) → PHA(2) → FLN(6) → CHO(3) → TRM(1) → DLY(4) → REV(8) → DAC(0)
```

---

## 2. Effect Slot Wrapper

**Source:** `src/effects/axis_effect_slot.sv`

Each slot wraps one effect core, selected at elaboration time by the `EFFECT_TYPE` parameter.

### EFFECT_TYPE Mapping

| Type | Effect | Slot | Port | Latency | Output |
|------|--------|------|------|---------|--------|
| 0 | Tremolo | 0 | 1 | 1 cycle | Mono |
| 1 | Phaser | 1 | 2 | 1 cycle | Mono |
| 2 | Chorus | 2 | 3 | 1 cycle | Stereo |
| 3 | Delay | 3 | 4 | 3 cycles | Mono |
| 4 | Tube Distortion | 4 | 5 | 1 cycle | Mono |
| 5 | Flanger | 5 | 6 | 1 cycle | Mono |
| 6 | Big Muff | 6 | 7 | 1 cycle | Mono |
| 7 | Reverb | 7 | 8 | 1 cycle | Stereo |
| 8 | Compressor | 8 | 9 | 1 cycle | Mono |
| 9 | Wah | 9 | 10 | 1 cycle | Mono |

### Bypass Crossfade

- 64-sample linear crossfade (~1.3 ms at 48 kHz)
- `fade_pos` counter ramps 0 → 64 (bypass → active) or 64 → 0 (active → bypass)
- Mix: `out = (dry × (64 - fade_pos) + wet × fade_pos) / 64`
- Effect always runs in bypass to keep LFO phase and delay tails alive

### Latency Handling

- All effects: `effect_valid = sample_en_d1` (1-cycle delay of `sample_en_reg`)
- `efx_valid` is the sole output strobe in all modes (bypass, active, crossfade)

### Config Slice

Each slot receives `cfg_slice[0:7]` — eight 8-bit registers from `ctrl_bus`:
```
cfg_slice = cfg_mem[SLOT_ID * 8 +: 8]
```
Parameters are always up-to-date one cycle after a `ctrl_bus` write. Bypass bit is `cfg_slice[7][0]`.

---

## 3. Effects

### 3.1 Tremolo

**Inspiration:** Classic AM tremolo
**Slot:** 0 | **Port:** 1 | **Source:** `src/effects/tremolo.sv`

#### Register Map

| Reg | Addr | CLI | Label | Default | Description |
|-----|------|-----|-------|---------|-------------|
| 0 | 0x00 | `rat` | Rate | 0x3C | LFO rate (~0.5–12 Hz) |
| 1 | 0x01 | `dep` | Depth | 0xB4 | Modulation depth (0=none, 0xFF=full chop) |
| 2 | 0x02 | `shp` | Shape | 0x00 | Waveform: bit 0 → 0=sine, 1=triangle |
| 7 | 0x07 | — | Bypass | 0x01 | bit 0: 0=active, 1=bypassed |

#### Signal Flow

```
                     ┌──────────┐
 rate_val ──────────►│ lfo_core │──► phase_out ──┐
                     │ TRIANGLE │                │
                     └──────────┘                │
                                          ┌──────▼───────┐
                          shape_sel ─────►│  Waveform    │
                                          │  Selection   │
                                          │ (tri/sine)   │
                                          └──────┬───────┘
                                                 │ lfo_norm (Q0.16 unsigned)
                              ┌──────────────────▼──────────────┐
               depth_val ────►│ gain = 1.0 - depth × (1 - lfo) │  Q1.16
                              └──────────────────┬──────────────┘
                                                 │
                    ┌──────────┐          ┌──────▼──────┐
   audio_in ───────►│  × gain  ├─────────►│  output reg │──► audio_out
                    └──────────┘          └─────────────┘
```

#### DSP Detail

- **LFO:** `lfo_core` with `WAVE_TYPE="TRIANGLE"`, `INC_BASE=44739`, `INC_SCALE=4035`, 8-bit table
- **Waveform selection:** triangle (direct from LFO) or sine (via `waveform_lut` quarter-wave ROM)
  - Sine path takes absolute value for unipolar hump: `lfo_norm = |wave_signed| × 2`
- **Gain calculation:** `gain = (1 << LFO_W) - depth_norm × ((1 << LFO_W) - lfo_norm)` (Q1.16)
- **AM multiply:** `audio_out = (audio_in × gain) >> LFO_W` (Q1.23 × Q1.16 → Q2.39 >> 16 → Q1.23)
- **Shared modules:** `lfo_core`, `waveform_lut` (SINE)
- **Latency:** 1 cycle

---

### 3.2 Phaser

**Inspiration:** MXR Phase 90 (Script/Block variants)
**Slot:** 1 | **Port:** 2 | **Source:** `src/effects/phaser.sv`

#### Register Map

| Reg | Addr | CLI | Label | Default | Description |
|-----|------|-----|-------|---------|-------------|
| 0 | 0x08 | `spd` | Speed | 0x50 | LFO rate (~0.5–5 Hz) |
| 1 | 0x09 | `fbn` | Feedback En | 0x00 | bit 0: 0=Script (no R28), 1=Block (feedback) |
| 7 | 0x0F | — | Bypass | 0x01 | bit 0: 0=active, 1=bypassed |

#### Signal Flow

```
                     ┌──────────┐
 speed_val ─────────►│ lfo_core │──► lfo_norm ──► lfo_sq (squared)
                     │ TRIANGLE │                      │
                     └──────────┘               ┌──────▼──────┐
                                                │ ap_coeff =  │
                                                │ C_MIN +     │
                                                │ C_SPAN×sq   │  Q0.16
                                                └──────┬──────┘
                                                       │
          ┌──────────────────────────────────────────── │ ──────────────┐
          │                                            │               │
          │   ┌───────┐    ┌───────┐    ┌───────┐    ┌─▼─────┐        │
 in ──+──►│──►│ AP[0] │───►│ AP[1] │───►│ AP[2] │───►│ AP[3] │──┐     │
      │   │   └───────┘    └───────┘    └───────┘    └───────┘  │     │
      │   │                                                      │     │
      │   │ feedback_en:  fb = sat(AP[3] × 0.5) ◄────────────────┘     │
      │   └────────────────────────────────────────────────────────────┘
      │
      └──► 50/50 dry+wet mixer ──► output reg ──► audio_out
```

#### DSP Detail

- **LFO:** `lfo_core` with `WAVE_TYPE="TRIANGLE"`, `INC_BASE=44739`, `INC_SCALE=1580`
- **Coefficient sweep:** `lfo_sq = lfo_norm²` (Q0.32), `ap_coeff = 137 + (2594 × lfo_sq) >> 16` (Q0.16)
  - C_MIN=137 (~lowest break freq), C_MAX=2731 (~highest break freq)
- **All-pass stage:** `y[n] = x[n] - c×x[n] - x[n-1] + y[n-1] - c×y[n-1]`
  - Accumulator width: `ACC_W = WIDTH + COEFF_W` bits, saturated at each stage output
- **Feedback (Block mode):** `fb = saturate(AP[3] >> 1)` summed with input
- **Output mix:** `(dry >> 1) + (wet >> 1)` (50/50 fixed)
- **Shared modules:** `lfo_core`, `saturate` (×4 stages + feedback)
- **Latency:** 1 cycle

---

### 3.3 Chorus

**Inspiration:** Boss CE-2 / Roland Dimension-D style
**Slot:** 2 | **Port:** 3 | **Source:** `src/effects/chorus.sv`

#### Register Map

| Reg | Addr | CLI | Label | Default | Description |
|-----|------|-----|-------|---------|-------------|
| 0 | 0x10 | `rat` | Rate | 0x50 | LFO rate |
| 1 | 0x11 | `dep` | Depth | 0x64 | Modulation depth (uses bits [7:1]) |
| 2 | 0x12 | `efx` | Effect | 0x80 | Wet level (0–255) |
| 3 | 0x13 | `eqh` | EQ High | 0xC8 | High-band EQ gain |
| 4 | 0x14 | `eql` | EQ Low | 0x80 | Low-band EQ gain |
| 7 | 0x17 | — | Bypass | 0x01 | bit 0: 0=active, 1=bypassed |

#### Signal Flow

```
                    ┌──────────┐
 rate_val ─────────►│ lfo_core │──► lfo_val (Q1.15 signed)
                    │   SINE   │         │
                    └──────────┘         │
                                  ┌──────▼───────┐
            depth ───────────────►│ mod_off =    │
                                  │ lfo × depth  │
                                  └──────┬───────┘
                                         │
              ┌────────────┐      ┌──────▼───────┐
 audio_in ───►│ delay_line │◄─────│  rd_ptr =    │
              │ DEPTH=2048 │      │ wr-960-mod   │
              └─────┬──────┘      └──────────────┘
                    │ delayed
              ┌─────▼──────┐
              │  2-band EQ │
              │  (biquad)  │
              │ LP + HP    │
              └─────┬──────┘
                    │ wet
    ┌───────────────┼───────────────┐
    │               │               │
  ┌─▼─┐    ┌───────▼───────┐     ┌─▼─┐
  │dry│    │  stereo split │     │dry│
  │×128    │ L = dry+wet   │     │×128
  └─┬─┘   │ R = dry-wet   │     └─┬─┘
    │      └───────┬───────┘       │
    └──────►  sat  ◄───────────────┘
           └───┬───┘
               ▼
          audio_out_l / audio_out_r
```

#### DSP Detail

- **LFO:** `lfo_core` with `WAVE_TYPE="SINE"`, `INC_BASE=8948`, `INC_SCALE=706880`, `INC_SHIFT=8`, 10-bit table
- **Delay modulation:** centre tap at 960 samples (20 ms), `mod_off = (lfo_val × mod_amp) >> LFO_FRAC`
  - Read pointer: `rd_ptr = wr_ptr_next - 960 - mod_off` (uses `wr_ptr_next` to avoid collision)
- **2-band EQ:** biquad crossover LPF (b0=0.125, a1_neg=0.875), high = delayed - LP
  - Each band scaled by EQ gain (unity at 128), products shifted >> 7
- **Stereo output:** `L = dry×128 + wet×effect_lvl`, `R = dry×128 - wet×effect_lvl`, both >> 8 with saturation
- **Shared modules:** `lfo_core`, `delay_line` (DEPTH=2048, 1 tap), `biquad_tdf2` (Q1.23), `saturate` (×2)
- **Latency:** 1 cycle

---

### 3.4 Delay

**Inspiration:** Strymon Timeline (Digital/Tape/Analog modes)
**Slot:** 3 | **Port:** 4 | **Source:** `src/effects/delay.sv`

#### Register Map

| Reg | Addr | CLI | Label | Default | Description |
|-----|------|-----|-------|---------|-------------|
| 0 | 0x18 | `rpt` | Repeats | 0x64 | Feedback gain (0=single echo, 0xFF≈infinite) |
| 1 | 0x19 | `mix` | Mix | 0x80 | Dry/wet crossfade (0=dry, 0x80=50/50, 0xFF=wet) |
| 2 | 0x1A | `flt` | Filter | 0xFF | Feedback LP cutoff (0=dark, 0xFF=bright) |
| 3-4 | 0x1B-1C | `tim` | Time | 0x3000 | 16-bit LE delay in samples (600–38400) |
| 5 | 0x1D | `mod` | Mod | 0x00 | LFO modulation depth (wow/flutter) |
| 6 | 0x1E | `grt` | Grit | 0x00 | [7:2]=saturation amount, [1:0]=mode |
| 7 | 0x1F | — | Bypass | 0x01 | bit 0: 0=active, 1=bypassed |

**Mode bits** (`grit_val[1:0]`):
- `00` = Digital — no modulation, no saturation
- `01` = Tape — full mod depth, grit saturation via `soft_clip`
- `10` = Analog — half mod depth, darker LP rolloff

#### Signal Flow

```
                    ┌──────────┐
 rate=0x40 ────────►│ lfo_core │──► lfo_wave (Q1.15)
                    │   SINE   │         │
                    └──────────┘         │ (mode-dependent scaling)
                                  ┌──────▼───────┐
             mod_val ────────────►│ lfo_offset   │  ±32 samples max
                                  └──────┬───────┘
                                         │
              ┌────────────┐      ┌──────▼───────┐
 audio_in ───►│ delay_line │◄─────│ rd_ptr =     │
              │ DEPTH=64K  │      │ wr-time+off  │
              │ REG_RD=1   │      └──────────────┘
              └─────┬──────┘
                    │ delayed (1-cycle read latency)
              ┌─────▼──────┐
              │  1-pole LP  │  alpha = f(filter_val, mode)
              │  damping    │
              └─────┬──────┘
                    │ filtered
              ┌─────▼──────┐   (tape mode + grit>0 only)
              │  grit boost│──► soft_clip
              │  × (1+g/16)│
              └─────┬──────┘
                    │ fb_source
              ┌─────▼──────┐
              │ × repeats  │──► + audio_in ──► sat ──► write back
              │  /256      │
              └─────┬──────┘
                    │ wet
              ┌─────▼──────┐
              │ crossfade   │  out = dry×(255-mix) + wet×mix, /256
              │ dry/wet mix │
              └─────┬──────┘
                    ▼
               audio_out
```

#### DSP Detail

- **LFO:** `lfo_core` with `WAVE_TYPE="SINE"`, `INC_BASE=22370`, `INC_SCALE=2018`, 8-bit table, fixed rate=0x40
- **Modulation depth by mode:** tape=full `mod_val`, analog=`mod_val/2`, digital=0
  - Offset: `(lfo_wave × depth) >> 18` → ±32 sample max
- **Tone control LP (1-pole):**
  - Digital/Tape: `alpha = (filter_val + 48) / 512`
  - Analog: `alpha = (filter_val/2 + 24) / 512` (darker rolloff)
  - Update: `lpf_state += alpha × (delayed - lpf_state)`, Q9 rounding
- **Tape saturation:** boost = `(grit_level + 16) / 16` (1.0× to ~4.9×) → `soft_clip` (2:1 compression)
- **Feedback:** `fb = fb_source × repeats / 256`, summed with input and saturated
- **Output mix:** true crossfade: `dry × (255-mix) + wet × mix`, /256 with rounding
- **Shared modules:** `lfo_core`, `delay_line` (DEPTH=65536, REG_RD=1), `soft_clip` (IN_W=26, COMP_SHIFT=1), `saturate` (×2)
- **Latency:** 3 cycles (registered BRAM read + LPF + output mix)

---

### 3.5 Tube Distortion

**Inspiration:** Cascaded triode preamp + Baxandall tone stack + push-pull power amp
**Slot:** 4 | **Port:** 5 | **Source:** `src/effects/tube_distortion.sv`

#### Register Map

| Reg | Addr | CLI | Label | Default | Description |
|-----|------|-----|-------|---------|-------------|
| 0 | 0x20 | `gai` | Gain | 0x60 | Cascaded pre-gain (0=clean, 0xFF=heavy saturation) |
| 1 | 0x21 | `bas` | Tone Bass | 0x80 | Bass LPF (0x80=flat) |
| 2 | 0x22 | `mid` | Tone Mid | 0x80 | Mid BPF (0x80=flat) |
| 3 | 0x23 | `tre` | Tone Treble | 0x80 | Treble HPF (0x80=flat) |
| 4 | 0x24 | `lvl` | Level | 0xC0 | Output attenuation |
| 7 | 0x27 | — | Bypass | 0x01 | bit 0: 0=active, 1=bypassed |

#### Signal Flow

```
                         Preamp Stage 1               Preamp Stage 2
 audio_in ──► HPF 80Hz ──► × gain1 ──► clip1 ──► LPF 8kHz ──► × gain2 ──► clip2 ──► LPF 10kHz
              (coupling)    (1-4×)      (2:1)    (smooth)       (1.5-16×)   (4:1)     (smooth)
                                                                                          │
                                                                                    ┌─────▼──────┐
                                                                                    │ level ×2   │
                                                                                    │ restore    │
                                                                                    └─────┬──────┘
                                                                                          │
                                                        ┌─────────────────────────────────┤
                                                        │                │                │
                                                  ┌─────▼──────┐  ┌─────▼──────┐  ┌──────▼─────┐
                                                  │ Bass LPF   │  │ Mid BPF    │  │ Treble HPF │
                                                  │ 250 Hz     │  │ 1 kHz      │  │ 4 kHz      │
                                                  └─────┬──────┘  └─────┬──────┘  └──────┬─────┘
                                                        │               │                │
                                                  ┌─────▼───────────────▼────────────────▼─────┐
                                                  │ dry + bg×bass + mg×mid + tg×treble         │
                                                  └─────────────────────┬───────────────────────┘
                                                                        │
                                                                  ┌─────▼──────┐
                                                                  │ Power Amp  │
                                                                  │ ×2 → clip3 │  (2:1, 0.75 FS cap)
                                                                  └─────┬──────┘
                                                                        │
                                                                  ┌─────▼──────┐
                                                                  │ × level    │
                                                                  │   /256     │
                                                                  └─────┬──────┘
                                                                        ▼
                                                                   audio_out
```

#### DSP Detail

- **Input HPF:** 2nd-order Butterworth at 80 Hz (Q4.20) — models coupling capacitor, removes DC/sub-bass
- **Gain stages (Q8.8, same formula as big_muff):**
  - Stage 1: `256 + (gain × 768) >> 8` → 1.0× to 4.0×
  - Stage 2: `384 + (gain × 3712) >> 8` → 1.5× to 16.0×
  - At low gain: clean → edge-of-breakup. At high gain: cascaded saturation
- **Soft-clip stages:**
  - Stage 1 (preamp triode 1): KNEE=0.5 FS, PEAK=max, COMP_SHIFT=1 (2:1) — gentle, wide
  - Stage 2 (preamp triode 2): KNEE=0.375 FS, PEAK=0.5 FS, COMP_SHIFT=2 (4:1) — tighter
  - Stage 3 (power amp): KNEE=0.5 FS, PEAK=0.75 FS, COMP_SHIFT=1 (2:1) — warm compression
- **Smoothing filters:** 2nd-order Butterworth LPFs (Q4.20)
  - Post-stage-1: 8 kHz (warmer, tames aliasing)
  - Post-stage-2: 10 kHz (same as big_muff)
- **Level restore:** ×2 shift after stage 2 (signal capped at 50% FS by clip2)
- **Tone stack:** 3 parallel `biquad_tdf2` filters (Q4.20)
  - Bass LPF: Fc=250 Hz, Butterworth
  - Mid BPF: Fc=1 kHz, Q=1.5
  - Treble HPF: Fc=4 kHz, Butterworth
  - Each band gain: `signed'(knob - 128)` → 9-bit signed, EQ range ±6 dB (product >>> 7)
  - Placed between preamp and power amp — tone controls shape distortion character
- **Power stage:** fixed ×2 gain → soft_clip (gentle 2:1, models push-pull power tubes)
- **Level scaling:** `× level / 256`, saturated
- **Shared modules:** `soft_clip` (×3), `biquad_tdf2` (×6, Q4.20), `saturate` (×8)
- **Latency:** 1 cycle (combinational gain/clip chains, single registered output)

---

### 3.6 Flanger

**Inspiration:** BBD-style flanger (MXR, EHX Electric Mistress)
**Slot:** 5 | **Port:** 6 | **Source:** `src/effects/flanger.sv`

#### Register Map

| Reg | Addr | CLI | Label | Default | Description |
|-----|------|-----|-------|---------|-------------|
| 0 | 0x28 | `man` | Manual | 0x80 | Base delay position |
| 1 | 0x29 | `wid` | Width | 0xC0 | LFO modulation depth |
| 2 | 0x2A | `spd` | Speed | 0x30 | LFO rate |
| 3 | 0x2B | `reg` | Regen | 0xA0 | Feedback (0x80=zero, 0xFF=+max, 0x00=-max) |
| 7 | 0x2F | — | Bypass | 0x01 | bit 0: 0=active, 1=bypassed |

#### Signal Flow

```
                    ┌──────────┐
 speed_val ────────►│ lfo_core │──► lfo_tri (Q0.8 unsigned)
                    │ TRIANGLE │         │
                    └──────────┘         │
                                  ┌──────▼───────┐
 manual_val, width_val ──────────►│ delay_time = │
                                  │ base + lfo   │  clamped [16..256]
                                  └──────┬───────┘
                                         │ {integer, 10-bit frac}
              ┌────────────┐      ┌──────▼───────┐
              │ delay_line │◄─────│ 2-tap read   │
  write ─────►│ DEPTH=256  │      │ + interp_frac│
              │ INTERP_EN=1│      └──────────────┘
              └─────┬──────┘
                    │ interp_out
              ┌─────▼──────┐
              │  BBD LPF   │  2nd-order Butterworth @ 6 kHz (Q2.22)
              │  (biquad)  │
              └─────┬──────┘
                    │ wet_filtered
              ┌─────▼──────┐
              │ × regen    │  Q1.7 signed (±0.99 max)
              │ + input    │──► soft_clip ──► delay write
              └─────┬──────┘
                    │
              ┌─────▼──────┐
              │ (dry+wet)/2│  50/50 mix
              └─────┬──────┘
                    ▼
               audio_out
```

#### DSP Detail

- **LFO:** `lfo_core` with `WAVE_TYPE="TRIANGLE"`, 28-bit phase accumulator
  - `lfo_tri` converted to Q0.8 unsigned (0–255)
- **Delay time:** `base = MIN_DELAY + manual × (MAX_DELAY - MIN_DELAY) / 256`
  - Modulated: `delay_time = base + (lfo_tri × width_val × range) >> shifts`, clamped [16..256]
  - 10-bit fractional part drives interpolation
- **Delay line:** `DEPTH=256` (PoT), `NUM_TAPS=2`, `INTERP_EN=1`, `FRAC_W=10`
  - Tap 0 = integer position, Tap 1 = integer+1, hardware interpolation
  - PoT depth: natural modular subtraction for rd_ptr wrapping (no branching)
- **BBD reconstruction LPF:** `biquad_tdf2` with Q2.22 coefficients (Butterworth Fc=10 kHz)
- **Feedback:** `regen_signed = regen_val - 128` (Q1.7), `fb = (wet × regen) >> 7`
  - Input to delay: `soft_clip(audio_in + fb)` with knee at 0.5 FS, 2:1 compression
- **Output:** `(dry + wet_filtered) >> 1` (50/50 fixed mix)
- **Shared modules:** `lfo_core`, `delay_line` (INTERP_EN=1), `biquad_tdf2` (Q2.22), `soft_clip`
- **Latency:** 1 cycle

---

### 3.7 Big Muff

**Inspiration:** Electro-Harmonix Big Muff Pi
**Slot:** 6 | **Port:** 7 | **Source:** `src/effects/big_muff.sv`

#### Register Map

| Reg | Addr | CLI | Label | Default | Description |
|-----|------|-----|-------|---------|-------------|
| 0 | 0x30 | `sus` | Sustain | 0x80 | Fuzz gain (0=clean, 0xFF=max) |
| 1 | 0x31 | `ton` | Tone | 0x80 | Tone sweep (0x00=bass, 0x80=scoop, 0xFF=treble) |
| 2 | 0x32 | `vol` | Volume | 0xA0 | Output volume |
| 7 | 0x37 | — | Bypass | 0x01 | bit 0: 0=active, 1=bypassed |

#### Signal Flow

```
              Stage 1                    Stage 2
 audio_in ──► × gain1 ──► clip1 ──► smooth1 ──► × gain2 ──► clip2 ──► smooth2
              (1-4×)      (4:1)    (10kHz LP)    (1.5-16×)   (4:1)   (10kHz LP)
                                                                          │
                                                                    ┌─────▼──────┐
                                                                    │ level ×2   │
                                                                    │ restore    │
                                                                    └─────┬──────┘
                                                                          │
                                                         ┌────────────────┼────────┐
                                                         │                │        │
                                                   ┌─────▼──────┐  ┌─────▼──────┐ │
                                                   │  Tone LP   │  │  Tone HP   │ │
                                                   │  1200 Hz   │  │  1000 Hz   │ │
                                                   └─────┬──────┘  └─────┬──────┘ │
                                                         │               │        │
                                                   ┌─────▼───────────────▼──────┐ │
                                            tone ──►│ LP×(255-tone) + HP×tone  │ │
                                                   │        /256              │ │
                                                   └─────────────┬────────────┘ │
                                                                 │              │
                                                           ┌─────▼──────┐       │
                                              volume ─────►│  × volume  │       │
                                                           │   /256     │       │
                                                           └─────┬──────┘       │
                                                                 ▼              │
                                                            audio_out           │
```

#### DSP Detail

- **Gain stages (Q8.8):**
  - Stage 1: `256 + (sustain × 768) >> 8` → 1.0× to 4.0×
  - Stage 2: `384 + (sustain × 3712) >> 8` → 1.5× to 16.0×
  - Combined max gain: ~64×
- **Soft-clip (both stages):**
  - KNEE = 0.375 FS (`3 × 2^(WIDTH-1) / 8`)
  - PEAK = 0.5 FS (`2^(WIDTH-2)`)
  - COMP_SHIFT = 2 → 4:1 compression above knee
- **Smoothing filters:** 2nd-order Butterworth LPF at 10 kHz (Q4.20, COEFF_W=24, FRAC=20)
  - b0=b1=b2=0.2202, a1_neg=0.3076, a2_neg=-0.1883
- **Tone section:** parallel LP (1200 Hz) + HP (1000 Hz) biquads
  - LP: b0=b1=b2=0.00554, a1_neg=1.7786, a2_neg=-0.8008
  - HP: b0=0.9116, b1=-1.8232, b2=0.9116, a1_neg=1.8153, a2_neg=-0.8310
  - Blend: `LP × (255 - tone) + HP × tone`, >> 8
- **Level restore:** ×2 shift before tone section (if not clipping)
- **Shared modules:** `soft_clip` (×2, COMP_SHIFT=2), `biquad_tdf2` (×4, Q4.20), `saturate` (×3)
- **Latency:** 1 cycle

---

### 3.8 Reverb

**Inspiration:** Freeverb (Schroeder-Moorer)
**Slot:** 7 | **Port:** 8 | **Source:** `src/effects/reverb.sv`

#### Register Map

| Reg | Addr | CLI | Label | Default | Description |
|-----|------|-----|-------|---------|-------------|
| 0 | 0x38 | `dec` | Decay | 0x80 | Comb feedback (0x00=short, 0xFF=long, ~0.5–0.98) |
| 1 | 0x39 | `dmp` | Damping | 0x60 | HF absorption (0x00=bright, 0xFF=dark) |
| 2 | 0x3A | `mix` | Mix | 0x80 | Dry/wet (0x00=dry, 0x80=equal, 0xFF=wet) |
| 3 | 0x3B | `pre` | Pre-delay | 0x20 | Pre-delay (0–255 samples, ~0–5.3 ms) |
| 4 | 0x3C | `ton` | Tone | 0x80 | Output tone LP (0x00=bright, 0xFF=dark) |
| 5 | 0x3D | `lvl` | Level | 0x80 | Output level |
| 7 | 0x3F | — | Bypass | 0x01 | bit 0: 0=active, 1=bypassed |

#### Signal Flow

```
                  ┌────────────┐
 audio_in ───────►│ pre-delay  │  0–255 samples
                  │ DEPTH=256  │
                  └─────┬──────┘
                        │ pre_out
        ┌───────────────┼───────────────────────────────────────┐
        │               │               │               │       │ ...×8
  ┌─────▼──────┐  ┌─────▼──────┐  ┌─────▼──────┐  ┌────▼─────┐│
  │ Comb 0     │  │ Comb 1     │  │ Comb 2     │  │ Comb 3   ││
  │ D=1557     │  │ D=1617     │  │ D=1491     │  │ D=1422   ││
  │ LP-damped  │  │ LP-damped  │  │ LP-damped  │  │ LP-damped││
  │ feedback   │  │ feedback   │  │ feedback   │  │ feedback ││
  └─────┬──────┘  └─────┬──────┘  └─────┬──────┘  └────┬─────┘│
        └───────────────┼───────────────┼───────────────┘      │
                        ▼ sum ÷ 8                              │
              ┌─────────┴──────────┐                           │
              │                    │                           │
     ┌────────▼─────────┐  ┌──────▼──────────┐                │
     │ 4× Allpass (L)   │  │ 4× Allpass (R)  │                │
     │ D={556,441,      │  │ D={579,464,     │                │
     │   341,225}       │  │   358,238}      │                │
     │ g=0.5            │  │ g=0.5           │                │
     └────────┬─────────┘  └──────┬──────────┘                │
              │                    │                           │
     ┌────────▼─────────┐  ┌──────▼──────────┐                │
     │ 1-pole tone LP   │  │ 1-pole tone LP  │                │
     └────────┬─────────┘  └──────┬──────────┘                │
              │                    │                           │
     ┌────────▼─────────┐  ┌──────▼──────────┐                │
     │  × level  (sat)  │  │  × level  (sat) │                │
     └────────┬─────────┘  └──────┬──────────┘                │
              │                    │                           │
              ▼                    ▼
         dry/wet mix L        dry/wet mix R ──► audio_out_l/r
```

#### DSP Detail

- **Pre-delay:** `delay_line` (DEPTH=256, REG_RD=0), 0–255 sample delay
- **8 parallel comb filters:** mutually prime depths for dense echo pattern
  - Depths: {1557, 1617, 1491, 1422, 1277, 1356, 1188, 1116}
  - 1-pole LP damping in feedback: `lpf = (1 - damp/256) × buf_out + (damp/256) × lpf_state`
  - Feedback gain: `128 + (decay × 123) >> 8` → Q0.9 (~0.50 to 0.98)
  - Write: `saturate(pre_out + fb_sat)`
- **Comb sum:** all 8 outputs summed then >> 3 (÷8)
- **4 series allpass diffusers (L and R):**
  - L depths: {556, 441, 341, 225}
  - R depths: {579, 464, 358, 238} (offset for stereo decorrelation)
  - Allpass: `out = -g×in + buf_out`, `buf[n] = in + g×buf_out` where g=0.5 (128/256)
- **Tone filter:** 1-pole LP: `y = (1 - tone/256)×x + (tone/256)×y_prev`
- **Level scaling:** `× level >> 7` with saturation
- **Dry/wet mix:** `dry × (256 - mix) + wet × mix`, >> 8 with saturation
- **Shared modules:** `delay_line` (×17: 1 pre + 8 comb + 4 AP-L + 4 AP-R), `saturate` (×20: 8 comb-fb + 8 comb-write + 2 level + 2 mix)
- **Latency:** 1 cycle (all delay reads combinational, registered output)
- **Resource estimate:** ~10 BRAM36 out of 135 on xc7a100t

---

### 3.9 Compressor

**Inspiration:** dbx 160 / Universal Audio 1176 style
**Slot:** 8 | **Port:** 9 | **Source:** `src/effects/compressor.sv`

#### Register Map

| Reg | Addr | CLI | Label | Default | Description |
|-----|------|-----|-------|---------|-------------|
| 0 | 0x40 | `thr` | Threshold | 0x60 | Compression threshold (0-255) |
| 1 | 0x41 | `rat` | Ratio | 0x40 | Compression ratio (0=1:1, 255=∞:1) |
| 2 | 0x42 | `atk` | Attack | 0x20 | Attack time (0=instant, 255=slow) |
| 3 | 0x43 | `rel` | Release | 0x40 | Release time (0=fast, 255=slow) |
| 4 | 0x44 | `mak` | Makeup | 0x40 | Makeup gain (0=0 dB, 255≈+12 dB) |
| 7 | 0x47 | — | Bypass | 0x01 | bit 0: 0=active, 1=bypassed |

#### Signal Flow

```
                   ┌──────────────┐
 audio_in ────────►│ |x| (abs)    │──► abs_in
                   └──────┬───────┘
                          │
                   ┌──────▼───────┐
                   │ Peak envelope│  attack/release with variable coefficients
                   │   follower   │
                   └──────┬───────┘
                          │ envelope
                   ┌──────▼───────┐
 threshold_val ───►│ threshold /  │──► compressed_gain (Q0.16)
 ratio_val ───────►│   envelope   │    blended with unity by ratio
                   └──────┬───────┘
                          │ target_gain
                   ┌──────▼───────┐
                   │ Smooth gain  │  1-pole filter on gain
                   └──────┬───────┘
                          │ smooth_gain
                   ┌──────▼───────┐
 audio_in ────────►│ × smooth_gain│
                   │   >> 16      │
                   └──────┬───────┘
                          │
                   ┌──────▼───────┐
 makeup_val ──────►│ × makeup     │  1.0× to ~4.0× (Q8.8)
                   │   >> 8       │
                   └──────┬───────┘
                          │
                   ┌──────▼───────┐
                   │  saturate    │──► audio_out
                   └──────────────┘
```

#### DSP Detail

- **Envelope:** peak follower with configurable attack/release (3-bit shift control, /1 to /8 for attack, /16 to /128 for release)
- **Gain computation:** `compressed_gain = threshold_scaled << 16 / envelope` (Q0.16)
- **Ratio blending:** `target = unity × (255-ratio)/256 + compressed × ratio/256`
  - ratio_val=0 → no compression, ratio_val=255 → full limiting
- **Gain smoothing:** 1-pole filter on gain for click-free transitions
- **Makeup gain:** `256 + makeup_val × 3` (Q8.8) → 1.0× to ~4.0×
- **Shared modules:** `saturate` (×1)
- **DSP cost:** ~3–4 DSP48E1
- **Latency:** 1 cycle

---

### 3.10 Wah

**Inspiration:** Dunlop Cry Baby / auto-wah
**Slot:** 9 | **Port:** 10 | **Source:** `src/effects/wah.sv`

#### Register Map

| Reg | Addr | CLI | Label | Default | Description |
|-----|------|-----|-------|---------|-------------|
| 0 | 0x48 | `frq` | Frequency | 0x80 | Manual frequency / LFO rate (0-255) |
| 1 | 0x49 | `res` | Resonance | 0x60 | Filter Q (0=mild, 255=near self-oscillation) |
| 2 | 0x4A | `sns` | Sensitivity | 0x80 | Auto-wah envelope sensitivity (0-255) |
| 3 | 0x4B | `mod` | Mode | 0x00 | 0=manual, 1=auto-wah, 2=LFO |
| 4 | 0x4C | `mix` | Mix | 0xFF | Dry/wet blend (0=dry, 255=full wet) |
| 7 | 0x4F | — | Bypass | 0x01 | bit 0: 0=active, 1=bypassed |

**Mode values:**
- `0` = Manual — frequency controlled directly by `freq_val`
- `1` = Auto-wah — envelope follower × sensitivity drives frequency
- `2` = LFO — sine LFO sweeps frequency, rate set by `freq_val`

#### Signal Flow

```
              ┌─────────── Mode Select ──────────────┐
              │                                       │
 freq_val ────┤ mode=0: direct mapping                │
              │                                       │
              │ mode=1:  ┌──────────────┐             │
 audio_in ───►├─────────►│ Envelope     │             │
              │          │ follower     │──► env      │──► f_coeff
 sensitivity ─┤          └──────────────┘   × sens   │    (Q2.14)
              │                                       │
              │ mode=2:  ┌──────────────┐             │
 freq_val ───►├─────────►│ lfo_core     │──► lfo     │
              │          │ SINE         │   scaled    │
              │          └──────────────┘             │
              └───────────────────────────────────────┘

                   ┌──────────────────────────┐
                   │ SVF (State Variable      │
 audio_in ────────►│  Filter)                 │
 f_coeff ─────────►│ hp = x - lp - d×bp      │
 d_coeff ─────────►│ bp += f × hp            │──► bp (bandpass = wah)
 (from resonance)  │ lp += f × bp            │
                   └──────────┬───────────────┘
                              │
                   ┌──────────▼───────┐
 audio_in ────────►│ dry/wet mix      │  dry×(255-mix) + wet×mix >> 8
 mix_val ─────────►│                  │
                   └──────────┬───────┘
                              │
                   ┌──────────▼───────┐
                   │   saturate       │──► audio_out
                   └──────────────────┘
```

#### DSP Detail

- **SVF topology:** `hp = x - lp - d×bp`, `bp += f×hp`, `lp += f×bp`
  - More efficient than biquad for swept filters (only 2 multiplies for f and d)
- **Frequency range:** F_MIN=859 (~400 Hz) to F_MAX=5324 (~2500 Hz) in Q2.14
  - Mapped: `f_coeff = F_MIN + (ctrl × F_RANGE) / 256`
- **Damping from resonance:** D_MAX=29491 (Q=mild) to D_MIN=819 (Q≈self-oscillation) in Q2.14
  - Higher resonance_val = lower damping = higher Q
- **Envelope follower (auto-wah):** fast attack (÷4), slow decay (÷1024, ~20 ms)
- **LFO:** `lfo_core` with `WAVE_TYPE="SINE"`, `INC_BASE=17895` (~0.2 Hz), `INC_SCALE=803406` (up to ~9 Hz)
- **Output mix:** `dry × (255 - mix) + wet × mix`, >> 8 with saturation
- **Shared modules:** `lfo_core` (SINE), `saturate` (×3)
- **DSP cost:** ~3–4 DSP48E1
- **Latency:** 1 cycle

---

## 4. Shared DSP Modules

### 4.1 lfo_core

**Source:** `src/dsp/lfo_core.sv`

Phase accumulator with configurable waveform output.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `PHASE_W` | 32 | Phase accumulator width |
| `CTRL_W` | 8 | Control knob width |
| `LFO_W` | 16 | Output width (signed Q1.15) |
| `INC_BASE` | 44739 | Base phase increment |
| `INC_SCALE` | 4035 | Rate scaling multiplier |
| `INC_SHIFT` | 0 | Right-shift on rate product |
| `TABLE_BITS` | 8 | Waveform LUT index bits |
| `WAVE_TYPE` | `"TRIANGLE"` | Waveform selection |

**Rate formula:** `phase_inc = INC_BASE + (rate_val × INC_SCALE) >> INC_SHIFT`

**Outputs:**
- `wave_signed` — Q1.(LFO_W-1) signed, range [−32767, +32767]
- `wave_unsigned` — unsigned offset by LFO_PEAK (0 to 65534)
- `phase_out` — raw phase accumulator

**Latency:** 1 cycle (phase register), waveform lookup is combinational

### 4.2 waveform_lut

**Source:** `src/dsp/waveform_lut.sv`

Compile-time waveform ROM, selected by `WAVE_TYPE` parameter.

| Waveform | Implementation | ROM Size |
|----------|---------------|----------|
| `"SINE"` | Quarter-wave symmetry ROM | TABLE_SIZE/4 entries |
| `"TRIANGLE"` | Computed from phase bits | No ROM |
| `"SAWTOOTH"` | Direct from phase bits | No ROM |
| `"SQUARE"` | MSB selection | No ROM |

- **Sine:** stores one quarter-wave (64 entries for TABLE_BITS=8), reconstructs full cycle via quadrant folding
- **Output:** Q1.(OUT_WIDTH-1) signed, range [−32767, +32767]
- **Fractional output:** `frac_out` provides sub-sample phase for interpolation
- **Latency:** 0 cycles (pure combinational)

### 4.3 delay_line

**Source:** `src/dsp/delay_line.sv`

Circular BRAM buffer with multi-tap reads and optional interpolation.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `DATA_W` | 24 | Sample width (signed) |
| `DEPTH` | 2048 | Buffer depth (power-of-2 or not) |
| `NUM_TAPS` | 1 | Number of simultaneous read ports |
| `REG_RD` | 0 | 0=combinational read, 1=registered (1-cycle latency) |
| `INTERP_EN` | 0 | Enable built-in linear interpolation |
| `FRAC_W` | 10 | Fractional delay precision |

**Write:** circular pointer auto-increments on `wr_en`. Exposes `wr_ptr_o` and `wr_ptr_next_o`.

**Read modes:**
- `REG_RD=0`: combinational — `rd_data = mem[rd_ptr]` same cycle
- `REG_RD=1`: registered — 1-cycle latency, captured on `rd_en`

**Interpolation (`INTERP_EN=1`):** `interp_out = rd_data[0] + frac × (rd_data[1] - rd_data[0]) / 2^FRAC_W`

**Wrapping:** handles both power-of-2 and non-power-of-2 depths.

### 4.4 biquad_tdf2

**Source:** `src/dsp/biquad_tdf2.sv`

Transposed direct-form II biquad filter.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `DATA_W` | 24 | Audio sample width |
| `COEFF_W` | 24 | Coefficient width |
| `FRAC` | 23 | Fractional bits in coefficients |

**Transfer function:** `H(z) = (b0 + b1·z⁻¹ + b2·z⁻²) / (1 + a1·z⁻¹ + a2·z⁻²)`

**Coefficient inputs:** `b0`, `b1`, `b2`, `a1_neg`, `a2_neg` (denominator pre-negated)

**State equations (per sample, when `en=1`):**
```
y_full = (x × b0 + s1) >> FRAC       // output with saturation
s1     = x × b1 + y_full × a1_neg + s2
s2     = x × b2 + y_full × a2_neg
```

- Accumulator width: `ACC_W = DATA_W + COEFF_W` (48 bits typical)
- Output saturated via `saturate` module to prevent wrapping
- **Latency:** 1 cycle

### 4.5 saturate

**Source:** `src/dsp/saturate.sv`

Signed N-bit to M-bit clamp via guard-bit check.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `IN_W` | 33 | Input width (signed) |
| `OUT_W` | 24 | Output width (signed) |

**Algorithm:** check if all `GUARD = IN_W - OUT_W` upper bits match the sign bit. If yes, pass through lower bits. If overflow, clamp to max positive or max negative.

- **Latency:** 0 cycles (pure combinational)

### 4.6 soft_clip

**Source:** `src/dsp/soft_clip.sv`

Piecewise-linear soft saturation.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `IN_W` | 26 | Input width (signed) |
| `OUT_W` | 24 | Output width (signed) |
| `COMP_SHIFT` | 1 | Compression: 1=2:1, 2=4:1 |
| `KNEE` | `2^(OUT_W-2)` | Threshold magnitude |
| `PEAK` | `2^(OUT_W-1)-1` | Maximum output magnitude |

**Algorithm:**
```
if |x| ≤ KNEE:  y = x                          (1:1 linear)
if |x| > KNEE:  y = KNEE + (|x| - KNEE) >> COMP_SHIFT   (compressed)
if y > PEAK:    y = PEAK                       (hard clamp)
```

- **Latency:** 0 cycles (pure combinational)

### 4.7 linear_interp

**Source:** `src/dsp/linear_interp.sv`

3-stage pipelined fractional sample interpolation.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `DATA_W` | 24 | Sample width (signed) |
| `FRAC_W` | 12 | Fractional delay width |

**Pipeline:**
| Stage | Operation |
|-------|-----------|
| 1 | `delta = y1 - y0` (DATA_W+1 bits) |
| 2 | `prod = delta × frac` (PROD_W bits) |
| 3 | `y_out = y0 + round(prod >> FRAC_W)` |

Rounding: adds `2^(FRAC_W-1)` before shift (half-LSB rounding).

- **Latency:** 3 cycles

### 4.8 tube_lut_rom

**Source:** `src/dsp/tube_lut_rom.sv`

4096-entry ROM with 12AX7 triode transfer function.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ADDR_W` | 12 | Address width (4096 entries) |
| `DATA_W` | 24 | Output width (signed Q0.23) |

**Transfer function (computed at elaboration):**
```
x = (i / 2048 - 1.0) × 2.5 + 0.20       // positive DC bias → even harmonics
y = 0.93 × tanh(x)                        // fundamental soft clip
  + 0.05 × tanh(1.7 × x² × sgn(x))       // 2nd harmonic distortion
  + 0.02 × tanh(3 × x)                    // 3rd harmonic grit
mem[i] = clamp(y, [-1, +1]) × 2^23        // scaled to Q0.23
```

- Registered output (synchronous BRAM read)
- **Latency:** 1 cycle

---

## 5. Control Interface & Register Map

### Full Address Map

| Address | Slot | Register | CLI Param |
|---------|------|----------|-----------|
| **Tremolo (slot 0)** | | | |
| 0x00 | 0 | Rate | `rat` |
| 0x01 | 0 | Depth | `dep` |
| 0x02 | 0 | Shape (bit 0) | `shp` |
| 0x07 | 0 | Bypass (bit 0) | — |
| **Phaser (slot 1)** | | | |
| 0x08 | 1 | Speed | `spd` |
| 0x09 | 1 | Feedback enable (bit 0) | `fbn` |
| 0x0F | 1 | Bypass (bit 0) | — |
| **Chorus (slot 2)** | | | |
| 0x10 | 2 | Rate | `rat` |
| 0x11 | 2 | Depth | `dep` |
| 0x12 | 2 | Effect level | `efx` |
| 0x13 | 2 | EQ High | `eqh` |
| 0x14 | 2 | EQ Low | `eql` |
| 0x17 | 2 | Bypass (bit 0) | — |
| **Delay (slot 3)** | | | |
| 0x18 | 3 | Repeats | `rpt` |
| 0x19 | 3 | Mix | `mix` |
| 0x1A | 3 | Filter | `flt` |
| 0x1B | 3 | Time (low byte) | `tim` |
| 0x1C | 3 | Time (high byte) | `tim` |
| 0x1D | 3 | Mod | `mod` |
| 0x1E | 3 | Grit + mode | `grt` |
| 0x1F | 3 | Bypass (bit 0) | — |
| **Tube Distortion (slot 4)** | | | |
| 0x20 | 4 | Gain | `gai` |
| 0x21 | 4 | Tone Bass | `bas` |
| 0x22 | 4 | Tone Mid | `mid` |
| 0x23 | 4 | Tone Treble | `tre` |
| 0x24 | 4 | Level | `lvl` |
| 0x27 | 4 | Bypass (bit 0) | — |
| **Flanger (slot 5)** | | | |
| 0x28 | 5 | Manual | `man` |
| 0x29 | 5 | Width | `wid` |
| 0x2A | 5 | Speed | `spd` |
| 0x2B | 5 | Regen | `reg` |
| 0x2F | 5 | Bypass (bit 0) | — |
| **Big Muff (slot 6)** | | | |
| 0x30 | 6 | Sustain | `sus` |
| 0x31 | 6 | Tone | `ton` |
| 0x32 | 6 | Volume | `vol` |
| 0x37 | 6 | Bypass (bit 0) | — |
| **Reverb (slot 7)** | | | |
| 0x38 | 7 | Decay | `dec` |
| 0x39 | 7 | Damping | `dmp` |
| 0x3A | 7 | Mix | `mix` |
| 0x3B | 7 | Pre-delay | `pre` |
| 0x3C | 7 | Tone | `ton` |
| 0x3D | 7 | Level | `lvl` |
| 0x3F | 7 | Bypass (bit 0) | — |
| **Compressor (slot 8)** | | | |
| 0x40 | 8 | Threshold | `thr` |
| 0x41 | 8 | Ratio | `rat` |
| 0x42 | 8 | Attack | `atk` |
| 0x43 | 8 | Release | `rel` |
| 0x44 | 8 | Makeup | `mak` |
| 0x47 | 8 | Bypass (bit 0) | — |
| **Wah (slot 9)** | | | |
| 0x48 | 9 | Frequency | `frq` |
| 0x49 | 9 | Resonance | `res` |
| 0x4A | 9 | Sensitivity | `sns` |
| 0x4B | 9 | Mode | `mod` |
| 0x4C | 9 | Mix | `mix` |
| 0x4F | 9 | Bypass (bit 0) | — |
| **Route table** | | | |
| 0x60 | — | route[0] (DAC sink) | — |
| 0x61 | — | route[1] (Slot 0 sink) | — |
| 0x62 | — | route[2] (Slot 1 sink) | — |
| 0x63 | — | route[3] (Slot 2 sink) | — |
| 0x64 | — | route[4] (Slot 3 sink) | — |
| 0x65 | — | route[5] (Slot 4 sink) | — |
| 0x66 | — | route[6] (Slot 5 sink) | — |
| 0x67 | — | route[7] (Slot 6 sink) | — |
| 0x68 | — | route[8] (Slot 7 sink) | — |
| 0x69 | — | route[9] (Slot 8 sink) | — |
| 0x6A | — | route[10] (Slot 9 sink) | — |

### Bypass Convention

- Address: `slot × 8 + 7` (0x07, 0x0F, 0x17, 0x1F, 0x27, 0x2F, 0x37, 0x3F, 0x47, 0x4F)
- Write `0x00` → active (bypass off)
- Write `0x01` → bypassed (bypass on)
- All effects default to bypassed at reset

### CLI Commands

Commands are processed by `cmd_proc.sv` over UART. Prompt: `gtfx>`

| Command | Syntax | Description |
|---------|--------|-------------|
| `set` | `set <efx> <param> <hex>` | Set parameter value |
| `set` | `set <efx> on\|off` | Enable/disable effect (bypass) |
| `con` | `con <src> <snk>` | Connect source port to sink port |
| `chain` | `chain <tag1> <tag2> ...` | Redefine signal chain order |
| `wr` | `wr <addr><data>` | Raw register write (hex, no spaces) |
| `;` | `;` | Request status dump |

**Effect tags:** `trm`, `pha`, `cho`, `dly`, `tub`, `fln`, `bmf`, `rev`, `cmp`, `wah`
**Port tags:** `adc` (port 0), `trm` (1), `pha` (2), `cho` (3), `dly` (4), `tub` (5), `fln` (6), `bmf` (7), `rev` (8), `cmp` (9), `wah` (10)

### CDC Mechanism

Write path from `sys_clk` (cmd_proc) to `clk_audio` (ctrl_bus):

```
cmd_proc writes addr+data → toggles cmd_wr_toggle
                                     │
     ┌─────── clk_audio domain ──────▼───────────────┐
     │  tog_s1 ← cmd_wr_toggle   (metastability)     │
     │  tog_s2 ← tog_s1          (safe)              │
     │  tog_prev ← tog_s2                            │
     │  tog_edge = tog_s2 ^ tog_prev  (edge detect)  │
     │                                                │
     │  if (tog_edge):                                │
     │    capture addr + data                         │
     │    assert wr_en for 1 cycle                    │
     └────────────────────────────────────────────────┘
```

`cmd_proc` waits 31 `sys_clk` cycles between toggles to guarantee capture.

---

## 6. Crossbar Routing

**Source:** `src/ctrl/axis_crossbar.sv`

11-port symmetric AXI-Stream crossbar parameterised by `N_PORTS=11` and `DATA_W=48`.

### Port Assignment

| Port | Master (source) | Slave (sink) |
|------|-----------------|--------------|
| 0 | ADC | DAC |
| 1 | Slot 0 (Tremolo) | Slot 0 input |
| 2 | Slot 1 (Phaser) | Slot 1 input |
| 3 | Slot 2 (Chorus) | Slot 2 input |
| 4 | Slot 3 (Delay) | Slot 3 input |
| 5 | Slot 4 (Tube) | Slot 4 input |
| 6 | Slot 5 (Flanger) | Slot 5 input |
| 7 | Slot 6 (Big Muff) | Slot 6 input |
| 8 | Slot 7 (Reverb) | Slot 7 input |
| 9 | Slot 8 (Compressor) | Slot 8 input |
| 10 | Slot 9 (Wah) | Slot 9 input |

### Route Register Semantics

`route[sink_port] = source_port` — which master feeds each slave.

**Slave mux:** `s_tdata[k] = m_tdata[route[k]]`
**Master tready:** `m_tready[j] = OR of all s_tready[k] where route[k] == j`

### Default Route Table

| Register | Value | Meaning |
|----------|-------|---------|
| route[0] | 8 | DAC ← REV |
| route[1] | 3 | TRM ← CHO |
| route[2] | 5 | PHA ← TUB |
| route[3] | 6 | CHO ← FLN |
| route[4] | 1 | DLY ← TRM |
| route[5] | 7 | TUB ← BMF |
| route[6] | 2 | FLN ← PHA |
| route[7] | 9 | BMF ← CMP |
| route[8] | 4 | REV ← DLY |
| route[9] | 0 | CMP ← ADC |
| route[10] | 0 | WAH ← ADC (bypassed) |

Yielding the chain: `ADC(0) → CMP(9) → BMF(7) → TUB(5) → PHA(2) → FLN(6) → CHO(3) → TRM(1) → DLY(4) → REV(8) → DAC(0)`

### `chain` Command

The `chain` command rewrites route entries to form a linear chain from the listed effects:

```
chain bmf tub dly rev
```

This sets: `ADC → BMF → TUB → DLY → REV → DAC`, with unlisted effects removed from the chain.

Route table addresses: `0x60` + port index (written via `wr` or `chain`/`con` commands).
