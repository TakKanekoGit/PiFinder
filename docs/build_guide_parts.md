# Build Guide - Printed Parts

- [Introduction and Overview](build_guide.md)
- [UI Board](build_guide_ui.md)
- [Part Printing and Prep](build_guide_parts.md)
- [Assembly](build_guide_assembly.md)

## Parts ID

Below is an image showing all the parts required except for a [mounting](build_guide_parts.md#mounting) piece, along with where threaded inserts fit in each.  

![Parts List](../images/build_guide/v1.4/build_guide_25.jpg)

The Back Plate piece comes in two versions, one for use with a PiSugar (PS) and one without.  The PiSugar piece moves the camera slightly outboard to make room for the PiSugar battery pack.  

Due to the use of edge inserts, these pieces can be assembled in either left, or right, handed configurations so you just need the one set of parts regardless of which side your focuser is facing.  In the assembly guide you'll find info about how to orient the pieces as you put them together. 

## Printing

These pieces will print without supports in the orientation shown on the photo.  I use 3 perimeter layers and 15% infill, but the pieces are not large and don't need to handle heavy forces so almost any print settings should work.

You may want to consider using a material other than PLA, as your PiFinder is likely to experience some sunlight in it's lifetime and PLA degrades under moderate heat and UV.  PETG or some co-polymer like NGen would be a good choice.

## Inserts

Only some holes receive inserts, the rest have M2.5 screws inserted through them into the inserts in other pieces.  The brass inserts used in this project are M2.5 x 4mm long.  There are some inserts that go into holes through the entire piece thickness, and some that go into blind holes in the edges.  The edge inserts are indicated in the image above with arrows.

The Bottom Plate, Shroud, Bezel and Camera Cover have no inserts in them at all.

Because I use a lot of these inserts, I use a tool to help seat them plumb into the parts,  but I've done plenty freehand and it's not overly difficult.  Use a temperature a bit below your normal printing temperature (for reference, I print NGen at 220c and use 170c for inserts) and give the plastic time to melt around them.  

![Insert Inserting](../images/build_guide/v1.4/build_guide_02.jpg)

You can see a closer view of the through and blind inserts below

![Insert Inserting](../images/build_guide/v1.4/build_guide_03.jpg) 

## Mounting

Most people will want to print a dovetail mount which fits into the finder shoe included on most telescopes.  In the case/mount folder you'll find several options to match the orientation of your finder shoe.  The PiFinder will work if it's not completely plumb, but it's easier to reach the buttons and read the screen if it's upright and facing the focuser.  Print the dovetail which most closely matches the position of your finder shoe using the image below as reference:

![Finder shoe angle](../images/finder_shoe_angle.png)

If you need more flexibility, there is also a go-pro compatible plate that will bolt into the bottom plate.  You'll need to add inserts into the bottom plate mounting footprint to use this option.

Once you've got all the parts printed and inserts inserted, you're ready to [assemble](build_guide_assembly.md)!