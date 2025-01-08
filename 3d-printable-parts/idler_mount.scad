$fn = 30;
LENGTH = 67;
WIDTH = 40;
THICK = 3;
screw_d = 5.2;

// OpenBuilds idler mount dimensions: 67x40 
// Holes X: 9,29,61 Y:10,20,30 

idler_mount();

module idler_mount(){
    difference(){
        union(){
            cube([LENGTH,WIDTH,THICK]);
                      
            
        }        
        
        // Screwholes
        translate([7,10,-0.1]) cylinder(THICK+1,d=screw_d);
        translate([7,20,-0.1]) cylinder(THICK+1,d=screw_d);
        translate([7,30,-0.1]) cylinder(THICK+1,d=screw_d);
        translate([38.8,10,-0.1]) cylinder(THICK+1,d=screw_d);
        translate([38.8,20,-0.1]) cylinder(THICK+1,d=screw_d);
        translate([38.8,30,-0.1]) cylinder(THICK+1,d=screw_d);
        translate([58.8,10,-0.1]) cylinder(THICK+1,d=screw_d);
        translate([58.8,20,-0.1]) cylinder(THICK+1,d=screw_d);
        translate([58.8,30,-0.1]) cylinder(THICK+1,d=screw_d);
        
        // Rounded corners
        translate([5,5,0]) rotate([0,0,180]) angledEnd(5);
        translate([5,WIDTH-5,0]) rotate([0,0,90]) angledEnd(5);
        translate([LENGTH-5,5,0]) rotate([0,0,270]) angledEnd(5);
        translate([LENGTH-5,WIDTH-5,0]) rotate([0,0,0]) angledEnd(5);
        
        
    }
}

module angledEnd(angleRadius) {
    difference(){
        cube([angleRadius,angleRadius,14]);
        translate([0,0,-0.5]) cylinder(h = 15, r = angleRadius, $fn = 30);
    }
}