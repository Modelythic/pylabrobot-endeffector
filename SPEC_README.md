I want to make it easy to create a wrapper around pylabrobots "workcell" visualizing interface, and we also want to keep the same abstraction layers they define within a workcell. 3d visualizations should be done using URDF files, which enable not only CAD models to be shown in a robot environment, but also a definition of the configurability of the robots motion.

There will be two "interfaces", 1 thats a 2d top down view, where a user can select from 7 kinds of labware:
Spincoater: 1 substrate spot
hotplate: 25 substrate spots, 5x5 grid
substrate tray: 25 substrate spots, 5x5 grid, by default, the URDF for each tray should have substrates on it
vials: 3 vial spots, 3x3 grid
characterization line: 1 substrate spot
obstructions: in 2d space, its just going to be a red rectangular object that a user can place on the workcell to indicate any space that should be identified as an obstruction that the robo arm should avoid. later in 3d space, the top and bottom height of obstruction can be defined so that obstructions dont have to be just items that are directly touching the floor
robo arm : just a rectangle representing the base of the arm, no need to draw linkages or anything
	the robo arm can be defined explicitly as a agilex piper connected to our custom end effector in piper_ros/src/piper_description/urdf/piper_description_with_endeffector.urdf

then once they've clicked the save button on their config, then the 2d view is transformed into a 3d viewer where the user can see all the substrates, labware, and robo arm and end effector in a full view.

by default it can be a square workcell, but size can be configurable and the 3d/2d view should adjust accordingly within reasonable limits. once the workcell is defined I should be able to drag components onto the workcell. I should be able to add multiple version of each as long as there is space on the workcell, and they should snap to the grid of the workcell (not exactly sure what that should be, open to a good starting point) as the labware is being dragged, a green semi-transparent overlay should appear over the empty spots available in the workcell to place the labware, only where there's actually enough space for the labware based on its defined dimensions.

the end effector is very modular. it has support for a camera, a modular gripper (for now basically a tweezer gripper is fine), and up to 4 modules around the diameter of the end effector. pipettes are the only module that can be added to the end effector for now, but plan for the ability to add other components down the line. also, a camera lense should be centered in the fingers of the gripper to give a "POV" view of the gripper. already defined in URDF

with the configured workcell, which should be able to be saved a preset, user should see the 3d isometric view of the workcell in the main view of the page, and there should be a right-side scrolling view that allows you to select from previously defined protocol scripts, or allow the user to create their own protocol script. for protocol scripts, the protocol scripts shown should be ones that are compatible with the current workcell, which just means that the hardware used by the protocol has to be some subset of the current workcell. workcell presets should be able to be changed with some kind of dropdown

when a protocol is selected, the user should have the option to either run the protocol on the system if the hardware is connected, otherwise simulate it in 3d thanks to the URDFs, with the ability to speed up the operations to see the operations over a longer time span. you can actually create simulations using URDFs pretty easily, which would make for a pretty easy way to give an amazing visual to customers.

I think this layer has to be built to work indepedently, but also very well integrated with any optimization/decision making code. for now it should be able to function as a standalone program

a great reference for what I have in mind can be found by scrolling down to the lumen PNP config video shown on the main site of this cool tool: https://www.opulo.io/products/lumenpnp

another similar reference would be like a 3d printing slicing software (bambu studio, cura, etc)

