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

module arm() {
    bulb_d = 32;
    arm_w = 14;
    mount_t = 5;
    arm_t = 10;
    m_d = 16.6; // distance between mounting holes
    m_o = (bulb_d - m_d) / 2; // offset from edge for mounting holes
    m_hole_d = 8; // center hole diameter
    cham_r = 5;
    arm_l = 180;
    connect_d = 24;
    bm_d = 12; // distance between body mounting holes
    inner1_arm_l = arm_l - bulb_d/2 - connect_d/2;
    inner2_arm_l = arm_l - bulb_d - connect_d;
    screw_hole_d = 5;
    screwhead_hole_d = 7.5;
    difference() {
        //cube([140, arm_w, mount_t]);
        union() {
            // motor mounting bulb
            translate([bulb_d/2, 0, 0])
                cylinder(d=bulb_d, h=mount_t);
            
            // main arm itself
            intersection(){
                difference() {
                    union() {
                        slot(arm_l, bulb_d, mount_t, center_y=true);
                        translate([bulb_d/2, -bulb_d/2, 0])
                            cube([inner1_arm_l-connect_d/4, bulb_d, arm_t]);
                    };
                    translate([bulb_d/2, 0, 0])
                        cylinder(d=bulb_d, h=arm_t);
                    translate([arm_l-connect_d/2, 0, 0]) {
                        cylinder(d=connect_d, h=arm_t);
                        // also disallow side on the outer half of the cylinder
                        #translate([0, -arm_w, 0])
                            cube([connect_d/2, arm_w*2, arm_t]);
                    }
                }
                // rectangularish version
                translate([(bulb_d-2)*0, (-arm_w/2+cham_r)*0, arm_t/2*0]) rotate([0, 90, 0]) {
                    linear_extrude(height = arm_l) {
                        base_w = arm_w-arm_t/2;
                        polygon([[0, base_w], [-(arm_w-base_w), arm_w],
                                 [-(arm_w-base_w)*2, base_w], [-(arm_w-base_w)*2, -base_w],
                                 [-(arm_w-base_w), -arm_w], [0, -base_w]]);
                    };
                }
            }
            // body connecting mount area
            translate([arm_l-connect_d/2, 0, 0])
                cylinder(d=connect_d, h=mount_t);
        }
        // motor mounting holes
        translate([m_o, 0, 0]) {
            cylinder(d=screw_hole_d, h=mount_t);
            translate([m_d, 0, 0])
                cylinder(d=screw_hole_d, h=mount_t);
            translate([m_d/2, -m_d/2, 0])
                cylinder(d=screw_hole_d, h=mount_t);
            translate([m_d/2, m_d/2, 0])
                cylinder(d=screw_hole_d, h=mount_t);
            translate([m_d/2, 0, 0])
                cylinder(d=m_hole_d, h=mount_t);
            // hole to allow screw head to come through part way
            cylinder(d=screwhead_hole_d, h=mount_t-2.5);
            translate([m_d, 0, 0])
                cylinder(d=screwhead_hole_d, h=mount_t-2.5);
            translate([m_d/2, -m_d/2, 0])
                cylinder(d=screwhead_hole_d, h=mount_t-2.5);
            translate([m_d/2, m_d/2, 0])
                cylinder(d=screwhead_hole_d, h=mount_t-2.5);
        }
        // body mounting holes
        translate([arm_l-connect_d/2, 0, 0]) {
            translate([bm_d/2, 0, 0])
                cylinder(d=screw_hole_d, h=mount_t);
            translate([-bm_d/2, 0, 0])
                cylinder(d=screw_hole_d, h=mount_t);
            translate([0, bm_d/2, 0])
                cylinder(d=screw_hole_d, h=mount_t);
            translate([0, -bm_d/2, 0])
                cylinder(d=screw_hole_d, h=mount_t);
        }
        
        // strengthening holes -- small enough they add density
        str_d = 2;
        inner_l = arm_l - bulb_d - connect_d;
        translate([bulb_d, 0, 0]) {
            translate([inner_l/6*1, 0, 0])
                cylinder(d=str_d, h=arm_t);
            translate([inner_l/6*2, 0, 0])
                cylinder(d=str_d, h=arm_t);
            translate([inner_l/6*3, 0, 0])
                cylinder(d=str_d, h=arm_t);
            translate([inner_l/6*4, 0, 0])
                cylinder(d=str_d, h=arm_t);
            translate([inner_l/6*5, 0, 0])
                cylinder(d=str_d, h=arm_t);
        }
    }

}

//bldc();
arm();
