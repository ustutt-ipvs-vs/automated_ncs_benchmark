height=300;
plate_thick = 2.5;
screw_d=3.2;

cable_tower();


module cable_tower(){
    difference(){
        union(){
            cube([30,30,height]);
            translate([-10,0,0]) cube([50,30,50]);
            translate([-50,0,0]) cube([130,60,plate_thick]);
            
            translate([-40,0,plate_thick]) rotate([0,12,0]) cube([30,30,195]);
            translate([40+30,30,plate_thick]) rotate([0,12,180]) cube([30,30,195]);
            translate([0,60,plate_thick]) rotate([0,10,270]) cube([30,30,175]);
                      
        }
    
    // Bottom cut
    translate([-50,-10,-20]) cube([130,80,20]);
        
    // Cable channel
    translate([10,-0.1,plate_thick]) cube([10,10,height]);
    translate([10,0,height-10]) cube([10,31,11]);
    translate([10,0,height-30]) rotate([45,0,0]) cube([10,31,11]);
        
    // Ziptie holes
    translate([-10,5,height-15]) rotate([0,90,0]) cylinder(h=50,d=5);
    translate([-10,25,height-5]) rotate([0,90,0]) cylinder(h=50,d=5);
    translate([-10,5,height-100]) rotate([0,90,0]) cylinder(h=50,d=5);
    translate([-100,5,height-200]) rotate([0,90,0]) cylinder(h=200,d=5);
    
    // Screwholes bottom
    translate([-40,15,-0.1]) cylinder(h=5, d=screw_d, $fn=30);
    translate([70,15,-0.1]) cylinder(h=5, d=screw_d, $fn=30);
    translate([-40,15,plate_thick]) cylinder(h=25, d=10, $fn=30);
    translate([70,15,plate_thick]) cylinder(h=25, d=10, $fn=30);
    
    translate([-20,45,-0.1]) cylinder(h=5, d=screw_d, $fn=30);
    translate([50,45,-0.1]) cylinder(h=5, d=screw_d, $fn=30);
    
        
    
    }  
}
