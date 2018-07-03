// prop radius ~128mm

module bldc() {
    cylinder(d=28, h=26);
}

// use similar to cube
module slot(l, d, h, center_x=false, center_y=false) {
    r = d/2;
    l2 = l - d;
    union() {
        if (center_x && center_y) {
            translate([-l2/2, -r]) cube([l2, r*2, h]);
            translate([l2/2, 0]) cylinder(r = r, h = h);
            translate([-l2/2, 0]) cylinder(r = r, h = h);
        } else if (center_x) {
            translate([-l2/2, 0]) cube([l2, r*2, h]);
            translate([l2/2, r]) cylinder(r = r, h = h);
            translate([-l2/2, r]) cylinder(r = r, h = h);
        } else if (center_y) {
            translate([r, -r]) cube([l2, r*2, h]);
            translate([l2+r, 0]) cylinder(r = r, h = h);
            translate([r, 0]) cylinder(r = r, h = h);
        } else {
            translate([r, 0]) cube([l2, r*2, h]);
            translate([l2+r, r]) cylinder(r = r, h = h);
            translate([r, r]) cylinder(r = r, h = h);
        }
    }
}

// nophead: http://forum.openscad.org/rounded-corners-td3843.html
module fillet_blo(r, h) {
    translate([r / 2, r / 2, h/2])
        difference() {
            cube([r + 0.01, r + 0.01, h + 0.01], center = true);
            translate([r/2, r/2, 0])
                cylinder(r = r, h = h + 0.02, center = true);
        }
}

module fillet_bro(r, h) {
    rotate([0, 0, 90]) fillet_blo(r, h);
}

module fillet_tlo(r, h) {
    rotate([0, 0, -90]) fillet_blo(r, h);
}

module fillet_tro(r, h) {
    rotate([0, 0, 180]) fillet_blo(r, h);
}

module fillet_tli(r, h) {
    rotate([0, 0, -90]) fillet_blo(r, h);
}

module fillet_tri(r, h) {
    rotate([0, 0, 180]) fillet_blo(r, h);
}

// SIKORSKY DBLN-526 AIRFOIL
// http://www.airfoiltools.com/airfoil/details?airfoil=dbln526-il
module airfoil(foil_h, arm_l, alpha) {
    pts = [
    [0.0000, 0.0000],
    [0.0125, 0.0362],
    [0.0250, 0.0491],
    [0.0500, 0.0679],
    [0.0750, 0.0823],
    [0.1000, 0.0952],
    [0.1500, 0.1152],
    [0.2000, 0.1302],
    [0.2500, 0.1430],
    [0.3000, 0.1530],
    [0.4000, 0.1668],
    [0.5000, 0.1698],
    [0.6000, 0.1668],
    [0.7000, 0.1530],
    [0.7500, 0.1430],
    [0.8000, 0.1302],
    [0.8500, 0.1152],
    [0.9000, 0.0952],
    [0.9250, 0.0823],
    [0.9500, 0.0679],
    [0.9750, 0.0491],
    [0.9875, 0.0362],
    [1.0000, 0.0000],

    [0.0000, 0.0000],
    [0.0125, -.0208],
    [0.0250, -.0298],
    [0.0500, -.0405],
    [0.0750, -.0483],
    [0.1000, -.0548],
    [0.1500, -.0637],
    [0.2000, -.0708],
    [0.2500, -.0751],
    [0.3000, -.0823],
    [0.4000, -.0887],
    [0.5000, -.0901],
    [0.6000, -.0887],
    [0.7000, -.0823],
    [0.7500, -.0751],
    [0.8000, -.0708],
    [0.8500, -.0637],
    [0.9000, -.0548],
    [0.9250, -.0483],
    [0.9500, -.0405],
    [0.9750, -.0298],
    [0.9875, -.0208],
    [1.0000, 0.0000]
    ];
    base_h = (0.1698 - -.0901);
    base_h_diff = (0.1698 - .0901);
    foil_w = foil_h / base_h;
    rotate([0, 0, 90 + alpha])
    scale([foil_w, foil_w, 1])
    translate([-0.5, -base_h_diff/2, 0])
    linear_extrude(height = arm_l) {
        polygon(pts);
    }
}

module arm() {
    bulb_d = 29;
    arm_w = 16;
    arm_t = 8;
    m_d = 16; // distance between mounting holes
    m_o = (bulb_d - m_d) / 2; // offset from edge for mounting holes
    m_hole_d = 7.7; // center hole diameter
    cham_r = 5;
    arm_l = 140;
    connect_d = 24;
    bm_d = 12; // distance between body mounting holes
    inner1_arm_l = arm_l - bulb_d/2 - connect_d/2;
    inner2_arm_l = arm_l - bulb_d - connect_d;
    screw_hole_d = 3.5;
    screwhead_hole_d = 6.0;
    difference() {
        //cube([140, arm_w, arm_t]);
        union() {
            // motor mounting bulb
            translate([bulb_d/2, 0, 0])
                cylinder(d=bulb_d, h=arm_t);
            
            // main arm itself
            intersection(){
                union() {
                    slot(arm_l, bulb_d, arm_t, center_y=true);
                    difference() {
                        translate([bulb_d/2, -bulb_d/2, 0])
                            cube([inner1_arm_l-connect_d/2, bulb_d, arm_t*2]);
                        translate([bulb_d/2, 0, 0])
                            cylinder(d=bulb_d, h=arm_t*2);
                        *translate([arm_l-connect_d/2, 0, 0])
                            cylinder(d=connect_d, h=arm_t*2);
                    }
                }
                // foil version
                translate([bulb_d/2, 0, arm_t/2+1.5]) rotate([0, 90, 0]) {
                    airfoil(arm_t-1, inner1_arm_l, 18);
                }
                // cylinder version
                *translate([bulb_d-2, -arm_w/2+cham_r, arm_t/2]) rotate([0, 90, 0]) {
                    cylinder(r=cham_r, h=arm_l);
                    translate([0, arm_w-cham_r*2, 0])
                        cylinder(r=cham_r, h=arm_l);
                    translate([-arm_t/2, 0, 0])
                        cube([arm_t, arm_w-cham_r*2, arm_l]);
                }
            }
            // body connecting mount area
            translate([arm_l-connect_d/2, 0, 0])
                cylinder(d=connect_d, h=arm_t);
        }
        // motor mounting holes
        translate([m_o, 0, 0]) {
            cylinder(d=screw_hole_d, h=arm_t);
            translate([m_d, 0, 0])
                cylinder(d=screw_hole_d, h=arm_t);
            translate([m_d/2, -m_d/2, 0])
                cylinder(d=screw_hole_d, h=arm_t);
            translate([m_d/2, m_d/2, 0])
                cylinder(d=screw_hole_d, h=arm_t);
            translate([m_d/2, 0, 0])
                cylinder(d=m_hole_d, h=arm_t);
            // hole to allow screw head to come through part way
            cylinder(d=screwhead_hole_d, h=arm_t-2.5);
            translate([m_d, 0, 0])
                cylinder(d=screwhead_hole_d, h=arm_t-2.5);
            translate([m_d/2, -m_d/2, 0])
                cylinder(d=screwhead_hole_d, h=arm_t-2.5);
            translate([m_d/2, m_d/2, 0])
                cylinder(d=screwhead_hole_d, h=arm_t-2.5);
        }
        // body mounting holes
        translate([arm_l-connect_d/2, 0, 0]) {
            translate([bm_d/2, 0, 0])
                cylinder(d=screw_hole_d, h=arm_t);
            translate([-bm_d/2, 0, 0])
                cylinder(d=screw_hole_d, h=arm_t);
            translate([0, bm_d/2, 0])
                cylinder(d=screw_hole_d, h=arm_t);
            translate([0, -bm_d/2, 0])
                cylinder(d=screw_hole_d, h=arm_t);
        }
    }

}

//bldc();
arm();
