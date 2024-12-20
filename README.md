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

# Links that might be helpful

https://ris.utwente.nl/ws/portalfiles/portal/97082423/PhD_Thesis_E.L._Firrao_Lorenzo.pdf
https://k6jca.blogspot.com/2016/01/antenna-auto-tuner-design-part-10-final.html
