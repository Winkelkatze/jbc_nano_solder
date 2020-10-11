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

front_plate_width = 2;
sides_width = 2;
sides_height = 35;
sides_offset = front_plate_width;

pcb_offset = [22.9, 22.6];
pcb_size = [70.9, 71.6] - pcb_offset;
// top, right, bottom, left
pcb_spacing = [4, 30, 1, 1];

bottom_overlap = 2.5;
bottom_height = 1.5;
bottom_overlap_width = 3;

connector_holes = [
[10, 23, 7.5/2],
[35, 23, 10/2]];

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
        base();
        display_cutout();
        encoder_hole();
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

front_plate();

//translate([0, 0, 40])
//    bottom_plate();