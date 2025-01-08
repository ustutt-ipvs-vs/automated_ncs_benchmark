$fn=30;

width=39+15;
length=65.5;
height=65;
screw_d=3.2;

sensor_mount();
translate([4,6,height-6]) sensor_cable_guide();


module sensor_cable_guide(){
    guide_length = 100; // Changing this value is not fully supported
    difference(){
        union(){
            translate([0,6,0]) rotate([15,0,0]) cube([10,guide_length,6]);
            translate([0,0,-34]) cube([10,6,40]);
            translate([0,0,-34]) cube([10,guide_length-64+6,6]);
            translate([0,6+guide_length-64,-34]) rotate([45,0,0]) cube([10,85,6]);
        }
        
        // Screwholes + nuts
        translate([5,-0.1,-20]) rotate([-90,0,0]) cylinder(h=10,d=screw_d);
        translate([5,4,-20]) rotate([-90,0,0]) cylinder(h = 4, r=3.2, $fn=6);
        translate([5,-0.1,-5]) rotate([-90,0,0]) cylinder(h=10,d=screw_d);
        translate([5,4,-5]) rotate([-90,0,0]) cylinder(h = 4, r=3.2, $fn=6); 
    
        
        // Filament hole
        translate([5,70,20]) rotate([-75,0,0]) cylinder(h=50,d=3.2);
        
        // Ziptie hole
        translate([-0.1,65,19.]) rotate([0,90,0]) cylinder(h=12,d=4);
           
    }
    
}
module sensor_mount(){
    difference(){
        union(){
            cube([length,width,5]);
            cube([length,5,height]);
            
            translate([0,10,5]) rotate([135,0,0]) cube([length,10,4]);
            
            // Stability sides
            translate([0,5,height]) rotate([22-180,0,0]) cube([3,25,height+10]);
            translate([length-3,5,height]) rotate([22-180,0,0]) cube([3,25,height+10]);
        }
    
        // Sensor Hole
        translate([length/2,-0.1,45]) rotate([-90,0,0]) cylinder(h=10,d=20.5);
        
        // Sensor Screw holes
        translate([length/2,-0.1,45+15]) rotate([-90,0,0]) cylinder(h=10,d=3.2);
        translate([length/2+12.99,-0.1,45-7.5]) rotate([-90,0,0]) cylinder(h=10,d=3.2);
        translate([length/2-12.99,-0.1,45-7.5]) rotate([-90,0,0]) cylinder(h=10,d=3.2);
        
        // Cable guide holes
        translate([9,-0.1,39]) rotate([-90,0,0]) cylinder(h=10,d=3.2);
        translate([9,-0.1,39+15]) rotate([-90,0,0]) cylinder(h=10,d=3.2);
        
    
        translate([0,15,0]){
        // Screwholes bottom
        translate([32.75,12.9,-0.1]) cylinder(h=10,d=5.2);
        translate([22.75,32.75,-0.1]) cylinder(h=10,d=5.2);      
        translate([42.75,32.75,-0.1]) cylinder(h=10,d=5.2);
        
        // Screwheads bottom
        translate([32.75,12.9,5]) cylinder(h=10,d=11);

        // Wheel Nut openings
        translate([12.75,12.9,-0.1]) cylinder(h=10,d=10);
        translate([length-12.75,12.9,-0.1]) cylinder(h=10,d=10);
            
        // 45° Cut bottom
        translate([0,18,-0.1]) rotate([0,0,45]) cube([30,20,6]);
        translate([length,18,-0.1]) rotate([0,0,45]) cube([20,30,6]);
            
        }

        
        //bottom and back cut
        translate([-0.1,-0.1,-length]) cube([length+1,width+1,length]);
        translate([-0.1,-length,-length]) cube([length+1,length,2*length]);
    
    }  
}

