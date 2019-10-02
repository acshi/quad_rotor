module motor_mount(mount_t) {
    m_d1 = 16.6; // distance between closer together motor mounting holes
    m_d2 = 19; // and between the farther apart ones
    m_hole_d = 8; // center hole diameter
    screwhead_hole_d = 6.5;
    screw_hole_d = 4.5;

    translate([0, 0, 0]) {
        rotate([0, 0, 45]) {
            translate([-m_d2/2, 0, 0])
                cylinder(d=screw_hole_d, h=mount_t);
            translate([m_d2/2, 0, 0])
                cylinder(d=screw_hole_d, h=mount_t);
            translate([0, -m_d1/2, 0])
                cylinder(d=screw_hole_d, h=mount_t);
            translate([0, m_d1/2, 0])
                cylinder(d=screw_hole_d, h=mount_t);
            translate([0, 0, 0])
                cylinder(d=m_hole_d, h=mount_t);
            // hole to allow screw head to come through part way
            translate([-m_d2/2, 0, 0])
                cylinder(d=screwhead_hole_d, h=mount_t-2.5);
            translate([m_d2/2, 0, 0])
                cylinder(d=screwhead_hole_d, h=mount_t-2.5);
            translate([0, -m_d1/2, 0])
                cylinder(d=screwhead_hole_d, h=mount_t-2.5);
            translate([0, m_d1/2, 0])
                cylinder(d=screwhead_hole_d, h=mount_t-2.5);
        }
    }
}


difference() {
    thick = 9;
    width = 32;
    height = 36;
    base = 50;
    union() {
        cube([thick, width, height]);
        cube([base, width, thick]);
    }
    translate([thick+0.001, width/2, height-12])
    rotate([180, 90, 0])
    motor_mount(thick+0.002);
}