$fn=30;

endstop_mount();

module endstop_mount(){
    screw_distance = 9.5;
    difference(){
        translate([-20,0,0]) cube([50,17,5]);
        
        // Extrusion mount
        translate([20,17/2,-0.1]) cylinder(h=6,d=3.2);
        translate([20,17/2-1.6,-0.1]) cube([5,3.2,6]);
        translate([25,17/2,-0.1]) cylinder(h=6,d=3.2);
        
        // Extrusion mount front
        translate([-10,17/2-4,-0.1]) cylinder(h=6,d=3.2);
        translate([-10-1.6,17/2-4,-0.1]) cube([3.2,8,6]);
        translate([-10,17/2+4,-0.1]) cylinder(h=6,d=3.2);
        
        // Endstop mount
        translate([7,(17-screw_distance)/2,-0.1]) cylinder(h=6,d=2.5);
        translate([7,17-(17-screw_distance)/2,-0.1]) cylinder(h=6,d=2.5);
        
        // Thinner part
        translate([11,-0.1,3]) cube([30,20,5]);
        translate([-20.1,-0.1,3]) cube([20.1,20,5]);
    }
}