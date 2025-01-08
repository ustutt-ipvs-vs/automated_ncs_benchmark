height = 5;

pcb_distance();

module pcb_distance(){
    difference(){
        union(){
            cylinder(h=height,d=6,$fn=30);
        }
        translate([0,0,-0.1]) cylinder(h=height+1,d=3.1,$fn=30);
    }
}