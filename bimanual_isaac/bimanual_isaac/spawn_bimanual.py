from pxr import UsdGeom
import omni.usd

# Updated imports from new namespace
from isaacsim.core.utils.prims import create_prim
from isaacsim.core.utils.stage import open_stage

def run():
    # Get current stage
    stage = omni.usd.get_context().get_stage()
    world_prim = stage.GetPrimAtPath("/World")

    # Don't recreate if already exists
    if not world_prim.IsValid():
        create_prim("/World", "Xform")

    # Example: Add a simple cube to /World
    cube_path = "/World/Cube"
    if not stage.GetPrimAtPath(cube_path).IsValid():
        cube_prim = UsdGeom.Cube.Define(stage, cube_path)
        cube_prim.AddTranslateOp().Set(value=(0.0, 0.0, 1.0))
        print("Cube added to stage")

    # Example: load a USD robot/environment if needed
    # open_stage("/path/to/your_robot_or_scene.usd")  # optional
    # OR use create_prim with "Xform" and then populate it with more logic

    print("Full script completed successfully")

run()
