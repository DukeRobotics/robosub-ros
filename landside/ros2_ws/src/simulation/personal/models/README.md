### Object IDs
Each object inside the simulation has a unique ID used to identify its type in order to determine what 
CV data to send. The IDs are:
- 1: Gate
    - Size: 60 x 120 in.
- 2: Buoy
    - size: 48x24 in. piece of paper, oriented facing sideways
- 3: Bin
    - Size: 36x26x8 in.
        - This is a GUESS!
        - 1/3 of the bin is covered by a 8x14x0.5 in. plate
        - 8 inches deep, 6 inch 'margins'
- 4: Octagon
    - Size: 2.7m diameter
- 5: Torpedo
    - Size: 2x2x6 in.
- 6: Path marker
    - Size: consists of two squirgles, each
    consisting of an 18 inch rectangle ending
    with a 3 inch radius circle. They are placed
    cocentric and rotated left or right by 30
    degrees relative to each other.
- 9: Gate side identifier, G-man (*Choose your side* abbreviated as CYS)
    - Size: 6x12 in. piece of paper facing sidways
- 10: Gate side identifier, Bootlegger (*Choose your side* abbreviated as CYS)
    - Size: 6x12 in. piece of paper facing sidways

### Objects not yet modelled:
- Left path marker
- Right path marker
- Bottle (Cash or Smash)
- Table (Cash or Smash)

### How to add a new object type
1. Define the ID that the object will use (for the "Object IDs" section), and 
ensure that no other object is already using this ID.
2. In your favorite CAD software (I'm using OnShape), make a quick approximation
of the object and export it as an STL file.
3. In CoppeliaSim, go to the Menu Bar and click File->Import->Mesh... and select the STL file. Click
'Import' in the resulting dialog box wtihout changing any settings.
4. With the imported object selected in the object hierarchy view,
 go to the Menu Bar and select Tools->Scene Object Properties.
5. (If the scale of the shape is wrong): , and in the properties popup window click 'View/modify geometry' near the bottom. From there you can modify the size of the object.
6. Go to the 'Common' tab in the Properties window, and ensure that the check
box titled 'Object is model base' toward the bottom.
7. Double click on the name of the object in the object hierarchy to give
it the appropriate name.
8. In the Menu Bar go to File->Save model as... and follow the resulting prompt
to save the model as a .ttm model in the personal/models/ folder.

You can now import the model to any scene using File->Load model...!