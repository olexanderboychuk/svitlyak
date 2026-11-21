// SVITLYAK / dimensional concept, not a fit-verified production enclosure.
// Units: mm. Verify every component against measurements before printing.
$fn = 96;
part = "assembly"; // assembly, base, lid, diffuser, exploded
body_d = 120;
body_h = 44;
wall = 3;
floor_t = 2.4;
lid_t = 3;
fit = 0.3; // radial clearance; tune for your printer
ring_od = 86; // PLACEHOLDER: 24 LEDs does not specify ring dimensions
ring_id = 70; // PLACEHOLDER
ring_t = 2;
pir_y = -76;
pir_pod_d = 28;
pir_lens_d = 10.5; // PLACEHOLDER: measure actual SR505 lens
usb_w = 14; // PLACEHOLDER: confirm connector location and size
usb_h = 8;
ldr_d = 5.5;
battery_l = 80; // PLACEHOLDER holder envelope, not bare cell dimensions
battery_w = 23;
esp_l = 58; // PLACEHOLDER ESP board envelope
esp_w = 31;
show_components = true;

assert(ring_od < body_d-20, "Increase body diameter for your LED ring");
assert(ring_id > 0 && ring_id < ring_od, "Invalid ring dimensions");
assert(body_h > 30, "Check battery holder and board stack height");

module outline(inset=0) {
  union() {
    circle(d=body_d-2*inset);
    translate([0,pir_y]) circle(d=pir_pod_d-2*inset);
    translate([0,(pir_y-48)/2]) square([24-2*inset,-48-pir_y],center=true);
  }
}
module annulus(od,id,h) {
  difference() {
    cylinder(d=od,h=h);
    translate([0,0,-0.1]) cylinder(d=id,h=h+0.2);
  }
}
module screw_locations() {
  for(a=[45,135,225,315])
    rotate([0,0,a]) translate([body_d/2-6,0,0]) children();
}
module base() {
  difference() {
    union() {
      difference() {
        linear_extrude(body_h) outline();
        translate([0,0,floor_t]) linear_extrude(body_h) outline(wall);
      }
      screw_locations() translate([0,0,floor_t]) cylinder(d=8,h=body_h-floor_t);
      // Short guide rails: secure the holder with a strap/tape, not exposed metal.
      for(y=[-20-battery_w/2-1,-20+battery_w/2+1])
        translate([-battery_l/2,y-1,floor_t]) cube([battery_l,2,5]);
      // Generic board supports; board attachment uses tape/adapter, not guessed holes.
      for(x=[-esp_l/2+4,esp_l/2-4])
        translate([x-2,18-esp_w/2,floor_t]) cube([4,esp_w,5]);
      // Inner opaque wall shields the LDR from direct LED-ring light.
      translate([body_d/2-14,-9,floor_t]) cube([2,18,body_h-floor_t-6]);
      for(y=[-10,8])
        translate([body_d/2-14,y,floor_t]) cube([14,2,body_h-floor_t-6]);
    }
    screw_locations() translate([0,0,body_h-14]) cylinder(d=2.5,h=15);
    // USB opening on upper side; the board/carrier must be aligned during fitting.
    translate([-usb_w/2,body_d/2-wall-2,10]) cube([usb_w,wall+5,usb_h]);
    // LDR views the room sideways. Seal around sensor leads with opaque material.
    translate([body_d/2-7,0,body_h-12]) rotate([0,90,0]) cylinder(d=ldr_d,h=12);
    // Two slots for a non-conductive battery-holder retaining strap.
    for(x=[-battery_l/2+10,battery_l/2-10])
      translate([x-2,-20-battery_w/2-3,-1]) cube([4,battery_w+6,floor_t+2]);
  }
}
module lid() {
  difference() {
    union() {
      linear_extrude(lid_t) outline();
      // Alignment lip, with radial printing clearance.
      translate([0,0,-2]) linear_extrude(2)
        difference() { outline(wall+fit); outline(wall+fit+1.2); }
    }
    // Through-window for the LED annulus; leave 3 bridges holding the centre disk.
    difference() {
      translate([0,0,-3]) annulus(ring_od,ring_id,lid_t+7);
      for(a=[30,150,270]) rotate([0,0,a])
        translate([ring_id/2-2,-1.2,-4]) cube([(ring_od-ring_id)/2+5,2.4,lid_t+10]);
    }
    // Recess for translucent insert, inset 1.2 mm from the front.
    translate([0,0,lid_t-1.2]) annulus(ring_od+4,ring_id-2,2);
    translate([0,pir_y,-3]) cylinder(d=pir_lens_d+0.4,h=lid_t+7);
    screw_locations() {
      translate([0,0,-3]) cylinder(d=3.2,h=lid_t+7);
      translate([0,0,lid_t-1]) cylinder(d=6,h=2);
    }
  }
}
module diffuser() {
  // Print separately in translucent material; no guarantee of optical uniformity.
  annulus(ring_od+4-2*fit,ring_id-2+2*fit,1.2);
}
module components() {
  // VISUAL ENVELOPES ONLY: not included in single-part exports.
  color([0.2,0.65,0.25,0.6])
    translate([-esp_l/2,18-esp_w/2,floor_t+5]) cube([esp_l,esp_w,7]);
  color([0.35,0.2,0.6,0.6])
    translate([-battery_l/2,-20-battery_w/2,floor_t]) cube([battery_l,battery_w,23]);
  color([0.05,0.35,0.2]) translate([0,0,body_h-ring_t-1]) annulus(ring_od,ring_id,ring_t);
  color("ivory") translate([0,pir_y,body_h+lid_t]) sphere(d=pir_lens_d);
}
if(part=="base") base();
else if(part=="lid") translate([0,0,2]) lid();
else if(part=="diffuser") diffuser();
else if(part=="assembly" || part=="exploded") {
  color([0.12,0.15,0.19]) base();
  if(show_components) components();
  lift = part=="exploded" ? 30 : 0;
  color([0.18,0.21,0.25]) translate([0,0,body_h+lift]) lid();
  color([1,0.8,0.4,0.7]) translate([0,0,body_h+lid_t-1.2+lift+(part=="exploded"?15:0)]) diffuser();
}
