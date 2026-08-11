# Enclosure

3D-printed two-piece case (magnet-held lid, no screws) sized for the "D1 Mini ESP32" board
(ESP32-WROOM-32 module, no mounting holes), the W5500 Ethernet breakout, and the SparkFun
MAX-M10S GPS module, as wired up in this project.

Designed in Fusion 360 (Personal), sliced/printed with Orca Slicer.

## Files

- `bottom-case.step` / `bottom-case.stl`
- `top-lid.step` / `top-lid.stl`

`.step` is the editable source (open in any CAD tool to modify); `.stl` is the print-ready mesh.

## Magnets

Both halves have embedded holes for 6x2mm magnets (6mm diameter, 2mm thick), used to hold the
lid closed:

- **Bottom case**: hole diameter 6.2mm, depth 2.4mm, 0.6mm of material remaining above the
  magnet cutout.
- **Top lid**: hole diameter 6.2mm, depth 2.4mm, 0.5mm of material remaining between the lid
  surface and the magnet cutout.

The magnets are embedded during printing, not press-fit afterward: when slicing, insert a pause
in the g-code at the layer where each cutout closes off, drop the magnet in, then resume -- the
print continues over the top of it. Check your slicer's docs for how to add a pause-at-layer/
pause-at-height correctly. Tested on a Bambu Lab X2D without issues, but your mileage may vary --
verify the pause height, the magnet fit tolerance, and consider a non-magnetic nozzle before
committing to a full print.

## License

Unlicense (public domain), matching the rest of this project.
