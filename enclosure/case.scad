$fn=32;

mounting_cone_hole = 0.5;
mounting_cone_hole_depth = 4;
mounting_cone_r_top = 2;
mounting_cone_r_bottom = 3;
mounting_cone_height = 8;

mounting_holes = [
[25.9, 25.1],
[35.8, 25.1],
[67.6, 25.1],
[25.1, 47.0],
[40.6, 45.2],
[26.9, 68.3]];

encoder_pos = [30.2, 35.9];
encoder_hole_r = 3.5;

disp_cutout = [25.5, 9];
disp_pos = [11 + 3 + encoder_pos[0], encoder_pos[1] - 0.8 - disp_cutout[1]/2];

stand_nuts = [[63, 35], [73, 45]];
stand_nuts_hole = 1.5;
stand_nuts_extra_strength = 1;
stand_nuts_socket_height = 2.5;
stand_nuts_socket_size = 3.1;
stand_nuts_socket_brim = 2;

front_plate_width = 2;
sides_width = 2;
sides_height = 35;
sides_offset = front_plate_width;

pcb_offset = [22.9, 22.6];
pcb_size = [70.9, 71.6] - pcb_offset;
// top, right, bottom, left
pcb_spacing = [4, 30, 1, 1];

connector_holes = [
[10, 23, 7.5/2],
[35, 23, 10/2]];

bottom_overlap = 2.5;
bottom_height = 1.5;
bottom_overlap_width = 3;

holder_mount_distance = stand_nuts[1] - stand_nuts[0];
holder_base_size = 20;
holder_base_height = 10;
holder_base_wire = 0.3;
holder_tower_r = 3.5;
holder_tower_height = 70;
holder_tower_hole = 1;
holder_screw_offset = 2;
holder_screw_head = 2.8;

holder_arm_height = 8;
holder_arm_length = 30;
holder_arm_tower_overlap_r = 2;
holder_arm_tower_overlap_h = 6;
holder_arm_circle = 8;
holder_arm_circle_inner = 5.5;
holder_arm_hole = 4;
holder_arm_wire = 0.8;


outer_a = [
        -pcb_spacing[3] - sides_width,
        -pcb_spacing[0] - sides_width];
    
outer_size = [
    pcb_size[0] + pcb_spacing[3] + pcb_spacing[1] + sides_width * 2,
    pcb_size[1] + pcb_spacing[2] + pcb_spacing[0] + sides_width * 2];


module base_plate()
{
    translate([-pcb_spacing[3], -pcb_spacing[0], 0])
    {
        cube([
            pcb_size[0] + pcb_spacing[3] + pcb_spacing[1],
            pcb_size[1] + pcb_spacing[2] + pcb_spacing[0],
            front_plate_width]);
    }
}

module mountings()
{
    translate([-pcb_offset[0], -pcb_offset[1], front_plate_width])
    {
        for (hole = mounting_holes)
        {
            translate([hole[0], hole[1], 0])
            {
                difference()
                {
                    cylinder(mounting_cone_height,
                             mounting_cone_r_bottom,
                             mounting_cone_r_top);
                    translate([
                        0,
                        0,
                        mounting_cone_height - mounting_cone_hole_depth])
                    {
                        cylinder(mounting_cone_hole_depth,
                                 mounting_cone_hole,
                                 mounting_cone_hole);
                    }
                }
            }
        }
    }
}

module display_cutout()
{
    translate([-pcb_offset[0] + disp_pos[0],
               -pcb_offset[1] + disp_pos[1],
               0])
    {
        cube([disp_cutout[0],
              disp_cutout[1],
              front_plate_width]);
    }
}

module encoder_hole()
{
    translate([-pcb_offset[0] + encoder_pos[0],
               -pcb_offset[1] + encoder_pos[1],
               0])
    {
        cylinder(h=front_plate_width,
                 r1=encoder_hole_r,
                 r2=encoder_hole_r);
    }
}

module sides()
{
    translate([outer_a[0], outer_a[1], sides_offset])
    {
        cube([outer_size[0], sides_width, sides_height]);
        cube([sides_width, outer_size[1], sides_height]);
    }
    
    translate([outer_a[0], outer_a[1] + outer_size[1] - sides_width, sides_offset])
    {
        cube([outer_size[0], sides_width, sides_height]);
    }
    
    translate([outer_a[0] + outer_size[0] - sides_width, outer_a[1], sides_offset])
    {
        cube([sides_width, outer_size[1], sides_height]);
    }
}

module connector_holes()
{
    translate([outer_a[0] + outer_size[0] - sides_width, 0, front_plate_width])
    {
        for (hole = connector_holes)
        {
            translate([0, hole[0], hole[1]])
            {
                rotate([0,90,0])
                {
                    cylinder(sides_width, hole[2], hole[2]);
                }
            }
        }
    }
}

module stand_nuts_pos()
{
    for (s = stand_nuts)
    {
        translate([s[0], s[1], front_plate_width])
        {
            cylinder(h = stand_nuts_socket_height, r = stand_nuts_socket_size + stand_nuts_socket_brim + stand_nuts_extra_strength, $fn=6);
        }
    }
}

module stand_nuts_neg()
{
    for (s = stand_nuts)
    {
        translate([s[0], s[1], 0])
        {
            cylinder(h = front_plate_width + stand_nuts_extra_strength, r = stand_nuts_hole);
        }
        translate([s[0], s[1], front_plate_width + stand_nuts_extra_strength])
        {
            cylinder(h = stand_nuts_socket_height, r = stand_nuts_socket_size, $fn=6);
        }
    }
}

module base()
{
    hull()
    {
        base_plate();
        translate([outer_a[0], outer_a[1], sides_offset - 0.01])
        {
            cube([outer_size[0], outer_size[1], 0.01]);
        }
    }
}

module front_plate()
{
    difference()
    {
        union()
        {
            base();
            stand_nuts_pos();
        }
        display_cutout();
        encoder_hole();
        stand_nuts_neg();
    }
    
    
    mountings();
    difference()
    {
        sides();
        connector_holes();
    }
}

module bottom_plate()
{
    
    translate([-pcb_spacing[3], -pcb_spacing[0], 0])
    {
        difference()
        {
            cube([
                pcb_size[0] + pcb_spacing[3] + pcb_spacing[1],
                pcb_size[1] + pcb_spacing[2] + pcb_spacing[0],
                bottom_overlap]);
            translate([bottom_overlap_width, bottom_overlap_width, 0])
            {
                cube([
                pcb_size[0] + pcb_spacing[3] + pcb_spacing[1] - bottom_overlap_width * 2,
                pcb_size[1] + pcb_spacing[2] + pcb_spacing[0] - bottom_overlap_width * 2,
                bottom_overlap]);
            }
        }
    }
    
    translate([outer_a[0], outer_a[1], -bottom_height])
    {
        cube([outer_size[0], outer_size[1], bottom_height]);
    }
}

module holder_tower()
{
    hull()
    {
        cube([holder_base_size, holder_base_size, 0.01]);
        translate([holder_base_size / 2, holder_base_size / 2, holder_base_height])
        {
            cylinder(h=0.01, r=holder_tower_r);
        }
    }
    translate([holder_base_size / 2, holder_base_size / 2, holder_base_height])
    {
        cylinder(h=holder_tower_height, r=holder_tower_r);
    }
}

module holder_tower_neg()
{
    translate([holder_base_size / 2, holder_base_size / 2, 0])
    {
        cylinder(h=(holder_base_height + holder_tower_height), r=holder_tower_hole);
        translate([-holder_mount_distance[0] / 2, -holder_mount_distance[1] / 2, 0])
        {
            cylinder(h=holder_screw_offset, r=stand_nuts_hole);
            translate([0, 0, holder_screw_offset])
            {
                cylinder(h=holder_base_height, r=holder_screw_head);
            }
        }
        translate([holder_mount_distance[0] / 2, holder_mount_distance[1] / 2, 0])
        {
            cylinder(h=holder_screw_offset, r=stand_nuts_hole);
            translate([0, 0, holder_screw_offset])
            {
                cylinder(h=holder_base_height, r=holder_screw_head);
            }
            
            k = stand_nuts_hole / sqrt(2);
            translate([-k, -k, 0])
            {
                cylinder(h=holder_screw_offset, r=holder_base_wire);
            }
        }
        rotate([0, 90, 45])
        {
            cylinder(h=norm(holder_mount_distance / 2), r=holder_base_wire);
        }
    }
}

module holder()
{
    difference()
    {
        holder_tower();
        holder_tower_neg();
    }
}

module holder_arm_pos()
{
    hull()
    {
        cylinder(h=holder_arm_height, r=(holder_arm_tower_overlap_r + holder_tower_r));
        translate([holder_arm_length, 0, 0])
        {
            cylinder(h=holder_arm_height, r=holder_arm_circle);
        }
    }
}

module holder_arm_neg()
{
    cylinder(h=holder_arm_tower_overlap_h, r=holder_tower_r);
    translate([holder_arm_length, 0, 0])
    {
        hull()
        {
            cylinder(h=holder_arm_height, r=holder_arm_hole);
            translate([holder_arm_length, 0, 0])
            {
                cylinder(h=holder_arm_height, r=holder_arm_hole);
            }
        }
    }
    translate([holder_arm_length, 0, 0])
    {
        cylinder(h=holder_arm_height, r=holder_arm_circle_inner);
    }
    
    translate([0, 0, holder_arm_tower_overlap_h])
    {
        rotate([0, 90, 0])
        {
            cylinder(h=holder_arm_length, r=holder_arm_wire);
        }
    }
}

module holder_arm()
{
    difference()
    {
        holder_arm_pos();
        holder_arm_neg();
    }
}

//front_plate();
/*
translate([58, 30, 0])
rotate([180, 0, 90])
{
    holder();
    translate([holder_base_size / 2, holder_base_size / 2, holder_tower_height + holder_base_height - holder_arm_tower_overlap_h])
    holder_arm();
}
*/
//translate([0, 0, 40])
//    bottom_plate();

holder_arm();