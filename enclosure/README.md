# Svitlyak enclosure concept

`svitlyak.scad` is a parametric OpenSCAD model. **Component dimensions are placeholders; measure your parts before printing a final enclosure.**

- Main body: Ø120 × 44 mm, 3 mm front cover, approximately 150 mm total height including the lower PIR pod.
- Assumed LED ring: Ø86/70 mm, 24 LEDs. LED count does not determine physical ring dimensions.
- PIR sits in the lower pod; orient the ESP antenna toward the opposite side.
- LDR looks sideways through an opening. Opaque internal partitions block direct ring light, but room reflections are still possible.
- Space is reserved for one 18650 holder and an ESP board. Supports are generic rather than matched to specific mounting holes.
- Charger/UPS dimensions and mounting are not defined. Simultaneous fit of all components has not been verified.
- PIR-to-ESP spacing is smaller than in the bench separation test. This enclosure does NOT guarantee immunity from RF interference.

## Export

In OpenSCAD, set `part = "base"`, `"lid"`, or `"diffuser"`, press F6, then export STL. `assembly` and `exploded` are viewing modes, not single-piece printable models.

The cover uses four M3 screws. Adjust the 2.5 mm pilot holes to your material and screws. Select screw length to avoid contact with electronics. Keep the cell and all electrical parts insulated from fasteners.

Prototype suggestion: opaque PETG, 3 mm walls, 0.2 mm layers. Orient the lid front-face down and check supports and the diffuser recess in your slicer. Print the diffuser separately in translucent material.

`concept.svg` is a design illustration, **not a product photograph or an exact CAD render**. A working version requires measurements of the ring, ESP, SR505, LDR, battery holder, and UPS board, plus checks for heating, battery access, and PIR interference. OpenSCAD rendering and physical fit have not yet been validated.
