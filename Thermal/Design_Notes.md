# Design Notes for Thermal Considerations
A compilation of thoughts and reasons for design decisions

## Camera Clamp
Goal is to design a press fit device to hold the camera steady inside a cylindrical housing. Must fit in a 78mm housing and hold a 30mm camera.

Trevor proposed a multipart angled design that forces parts outwards when tightened by means of forcing the wedges against each other. Simple design but requires a compression load to cause a shear force and shear movement.

Initial design aimed to implement a compliant mechanism to expand by the driving of a wedge into a slot. This would avoid the shear forces generated in Trevor's design. An axisymetric design with six wedges was chosen. Parametric modeling was attempted with all dimensions based on the values in the camera clamp parameters list. Initial 3D print with captured nuts was sucessful and worked to hold fast inside housing. Would be nice to reduce number of parts. A tapered screw could replace the need for a wedge.

Next iteration reduced to a slot for a countersunk screw to force the edges apart. I wanted to limit the number of screw to no more than six for the whole assembly. There was also concern raised that trying to force a rounded part to spread out was unecessarily hard. This can be avoided by using a straig slot, forced outwards by two screws per flap. It was tried with different sized slots and tapers, with the 30 \degree taper using countersunk M4 screws with sucess.

Per Rob and Niall, a 4mm slot is about the smallest that they can feasibly machine so the slot was increased. To accomodate this, I chose to size up to M5 countersunk screws.

FEA shows that we almost always stay within the yeild strength of aluminum however there were a few concentrations right on the edges of the screw holes that could be of concern. If there is plastic deforamtion/enough elastic deformation there, it could impact the threads of thd hole and bind the screw in the clamped position.

As of 6/25 - file sent to Rob for machining when possible.


## End Cap

The End cap file was provided by Trevor and the goal was to improve the thermal performance of it while maintaining the structural integrity. Fins of may different dimenstions were tested, with widths and gaps of 10, 5 and 4 mm were used a different depths. FEA performed with a hydrostatic load of 50000mm in -z direction and with convection with h = 50 on top surface. Constraint of rigid edge defined where the cap connects with the delrin. Appropriate heat flux added to the inside of the cap. Files were named based on the convention Width x Gap x Depth.

The best result was a 4mm by 4mm fins of 3mm deep. See notebook for thermal and stress results. Adding fillets to the bottom of the fins significnatly reduced thermal capability. Per discussion with Trevor, decision made to extrude an additonal 3mm to the cap such that the fins do not take away any structural integrity. No fillet was used by a 2mm radius curvature was added to the stopped cuts to allow for the mill bits. 

The 3 print of that model reveals that a) removing that much support is annoying, and that sharp edges will need to be considered and either chamfered away or removed from the design.