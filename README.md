## quickTuneHF

quickTuneHF was a project started about two years ago. The goal was to design alternative to ATU-100 that would withstand about 300W and the code would be Arduino based & opensource to make further modifications much easier. To further expand its capabilites, the CPU board is split form RF board - so the whole project might be rewritten to eg ESP32.
Unfortunately due to lack of time & issues with tuning algorithm this project is not fully operational.  

## BOM

Besides capacitors/resistors/diodes you might need:
* 2x Amidon T157-2
* 18x JQX115 relays
* HC4060
* ATMEGA328p
* 2x 28B0570-000 (for voltage transformer)
* 1x 5943004901 (current transformer)
* 2x TLC59213
* 2x 74HCT595

## Frequency meter

The frequency meter is based on PA2OHH work. You can check its schematic here:
![Freq meter](imgs/freq_meter.gif?raw=true "Schematic")

## Algorithm

Took K6JCA suggestion to tune inductor to the lowest SWR first, and then choose capacitor side. I was thinking about putting SGC-230 detection circuit, which was working quite well while testing, but unfortunately i had limited space - if i get the code working (maybe someday...) then i might go with air inductors and better sensing circuit.

## SWR diode nonlinearity correction

Schottky diodes used as envelope detectors in directional couplers do not have a
linear transfer characteristic.  At low RF levels they operate in the **square-law
region** where the detected DC voltage is proportional to the square of the RF
voltage (V_dc ∝ V_rf²).  At higher levels they transition toward linear detection.
Because the SWR formula

```
SWR = (1 + ref/fwd) / (1 - ref/fwd)
```

depends on the *voltage* ratio, applying the raw ADC readings directly will give
incorrect results whenever the two detectors are in different operating regions or
when the response is significantly nonlinear.  The correction is applied
identically to both channels before the ratio is computed, so only the shape of
the curve matters — not the absolute scale.

### Choosing a method

Set `SWR_CORRECTION_TYPE` near the top of `src/main.cpp`:

| Value | Method | When to use |
|---|---|---|
| `SWR_CORRECTION_NONE` | No correction | Raw ADC only; useful for debugging |
| `SWR_CORRECTION_LUT` | Lookup table + linear interpolation | Best accuracy after calibration |
| `SWR_CORRECTION_SQRT` | `corrected = sqrt(raw × 1023)` | Good starting point, no calibration needed |

### Square-root correction (`SWR_CORRECTION_SQRT`)

Assumes both detectors are fully in the square-law region:

```
V_rf ∝ sqrt(V_dc)   →   corrected = sqrt(raw × 1023)
```

The factor 1023 normalises the output so that full-scale input maps to full-scale
output.  No calibration is required.  This is a reasonable approximation at low to
medium power levels with typical Schottky diodes (1N5711, BAT43, etc.).

### Lookup table correction (`SWR_CORRECTION_LUT`)

A 33-entry table covers ADC values 0–1023 in steps of 32.  Values between
breakpoints are found by linear interpolation.  The table is stored in program
flash (`PROGMEM`) so it uses **no SRAM**.

Default entries are the square-law curve (`sqrt(raw × 1023)`).  To calibrate for
your actual diodes:

1. Connect a 50 Ω dummy load and apply RF at a known power level.
2. Read the raw FWD and REF ADC values over Serial (enable `DEBUG`).
3. The corrected values for both channels should be equal (ratio = 1, SWR = 1.0).
   If they are not, scale the REF channel entries up or down until the ratio
   matches.
4. Repeat at several power levels spread across your operating range.
5. Fill in `swr_lut[]` at the corresponding breakpoints (`index = raw / 32`).

Example — if at raw = 256 your REF reads 10 % low compared to FWD, increase
`swr_lut[8]` (breakpoint at 256) by ~10 %.

# Links that might be helpful

https://ris.utwente.nl/ws/portalfiles/portal/97082423/PhD_Thesis_E.L._Firrao_Lorenzo.pdf
https://k6jca.blogspot.com/2016/01/antenna-auto-tuner-design-part-10-final.html
