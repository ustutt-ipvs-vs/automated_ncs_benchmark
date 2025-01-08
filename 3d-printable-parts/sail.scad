$fn = 30;
SAIL_THICKNESS = 1.6;
SAIL_WIDTH = 141.4;
SAIL_HEIGHT = 141.4;
FRAME_WIDTH = 3;
FRAME_THICK = 4;

HOLDER_HEIGHT = SAIL_HEIGHT;

rotate([90,0,90]) sail();


module sail(){
    difference(){
        union(){
            translate([20,0,0]) cube([SAIL_THICKNESS,SAIL_HEIGHT,SAIL_WIDTH]);
            
            // Holder
            translate([0,0,0]) cube([25,HOLDER_HEIGHT,2]);
            translate([0,0,2 + 6]) cube([25,HOLDER_HEIGHT,2.4]);
            translate([20,0,0]) cube([5,HOLDER_HEIGHT,2 + 6 + 2.4]);
            
            translate([20.3,0,15]) rotate([0,135,0]) cube([7,HOLDER_HEIGHT,6.6]);
            
            // Frame
            translate([20-(FRAME_THICK-SAIL_THICKNESS)/2,0,0]) cube([FRAME_THICK,SAIL_HEIGHT,FRAME_WIDTH]);
            translate([20-(FRAME_THICK-SAIL_THICKNESS)/2,0,SAIL_WIDTH-FRAME_WIDTH]) cube([FRAME_THICK,SAIL_HEIGHT,FRAME_WIDTH]);
            
            
        }
        
        // Bottom flat
        translate([-1,-1,-10]) cube([102,102,10]);
        translate([0,-0.1,2]) cube([20,HOLDER_HEIGHT+1,6]);
        
        // Screwholes
        translate([10,10,5]) cylinder(10,d=2.5);
        translate([10,40,5]) cylinder(10,d=2.5);
        
        translate([10,HOLDER_HEIGHT-10,5]) cylinder(10,d=2.5);
        
        
    }
}