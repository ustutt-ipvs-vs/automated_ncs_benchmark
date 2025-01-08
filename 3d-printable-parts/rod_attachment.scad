$fn = 30;
THICK = 7;

rod_attachment_holder();


module rod_attachment_holder(){
    difference(){
        union(){
            cylinder(h=THICK,d=12);
            translate([0,-3,THICK/2]) rotate([90,0,0]) cylinder(h=3.5,d=4);
            
            // Pole holder
            translate([-20,-6,0]) cube([70,12,4]);
        }
        
        translate([0,0,-0.1]) cylinder(h=THICK+1,d=6.1);
        
        // Tightening screw
        translate([0,0,THICK/2]) rotate([90,0,0]) cylinder(h=10,d=2.6);
        
        // Pole attachment screws
        translate([-13,0,-0.1]) cylinder(h=THICK+1,d=3.5);
        translate([15,0,-0.1]) cylinder(h=THICK+1,d=3.5);
        translate([40,0,-0.1]) cylinder(h=THICK+1,d=3.5);
    }
}

