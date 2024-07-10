# Design Notes for Thermal FEA
A compilation of thoughts and comments from the process of FEA for thermal purposes

## Inventor Stress Analysis
Basic FEA only for stress and deformation built into Inventor. Environments > Stress Study.

## NASTRAN
A FEA solver integrated into Inventor. Must be downloaded and installed seperately but then accessed from Inventor. NASA STRuctural ANalysis. Environment > Autodesk Inventor Nastran. Good for solving stress and deformation problems. Has some built in thermal solving, but is not suitable for complex thermal problems involving flow or fluids.

**Notes:**
- Static Thermal Study includes conduction. Radiation and Convection must be added in as appropriate. Requires fixed h value
- Convection seems to be exclusively based on surface area and not consider geometry or flow at all
- Heat flux/generation must be expressed by unit volume/area
- Heat study and deformation/stress study cannot be combined. However, results of thermal study can be imported as a load into a stress study

**Tips:**
- Select material by Tools > Material and assigning to selection
- Define fixed points by where they connect to something else. e.g. bolt holes
- Surfaces must be added individually, does not require Ctrl held though
- Temperatures given in Kelvin
- Default view for results is an exagerated deformation that is not reflective of the actual deformation results. Can be helpful to turn this off and just look at the color bar


## CFD Ultimate
Very powerful (and expensive) FEA engine for solving flow related problems. Downloaded and installed seperately and used as a standalone program. Not suitable for stress/deformation, but very good for thermal or flow problems.

**Notes:**
- Can take inputs of most CAD files, does not seem to be many differences
- Must redefine materials once loaded in CFD regardless of materials set in CAD software
- To model convection:
    - use bouding volume significantly larger than the item 
    - Set input and output pressures to 0 and set initial temperature at input
    - Set material enviornment to be variable for fluid
- Autosize mesh is a pretty good starting spot but should be refined and made more fine in critical areas
- Geometry updates to similar models are easy to upload as an update to design and will preserve most materials and conditions

**Tips:**
- Simplify geometry significantly before importing. SimStudios has been discontinued. Inventor simplify feature not very goods so might be easiest to just reCAD as a primative from scratch
- Ctrl + MMB to hide/unhide components. Ctrl + wheel scroll can cycle through the components hidden in that view
- Planes and Iso Volumes are useful for visualizing results, right click on colorbar to set limits
- Can visualize wire mesh in results to see if the mesh is approrpriate for expected interactions