width=18;
length=15;
screw_d=3.2;

corner();


module corner(){
    difference(){
        union(){
            cube([length,width,3]);
            cube([3,width,length]);
            
            translate([0,0,3.5]) rotate([0,45,0]) cube([4,width,3]);
            translate([3,0,length]) rotate([0,135,0]) cube([length-3,2,sqrt(2*(length-3)*(length-3))]);
            translate([3,width-2,length]) rotate([0,135,0]) cube([length-3,2,sqrt(2*(length-3)*(length-3))]);
        }
    
    
    // Screwholes bottom
    translate([length-7,width/2-screw_d/2,-0.1]) cube([3,screw_d,5]);
    translate([length-7,width/2,-0.1]) cylinder(h=5, d=screw_d, $fn=30);
    translate([length-4,width/2,-0.1]) cylinder(h=5, d=screw_d, $fn=30);
    
    // Screwholes side
    translate([-0.1,width/2-screw_d/2,length-7]) cube([5,screw_d,3]);
    translate([-0.1,width/2,length-7]) rotate([0,90,0]) cylinder(h=5, d=screw_d, $fn=30);
    translate([-0.1,width/2,length-4]) rotate([0,90,0]) cylinder(h=5, d=screw_d, $fn=30);
        
    //bottom and back cut
    translate([-0.1,-0.1,-length]) cube([length,width+1,length]);
    translate([-length,-0.1,-0.1]) cube([length,width+1,length+1]);
    
    }  
}
