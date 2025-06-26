from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

# Your logic...
print("Script running inside Isaac Sim...")

from pxr import Gf, UsdGeom
from isaacsim.core.utils.prims import create_prim, get_current_stage, get_prim_at_path

def set_transform(path, translation):
    stage = get_current_stage()
    prim = stage.GetPrimAtPath(path)
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*translation))

def run():
    # Only create /World if it doesn't exist
    if not get_prim_at_path("/World").IsValid():
        create_prim("/World", "Xform")
        
    stage = get_current_stage()

    arm_usd = "/home/adityanair/assets/franka.usd"
    hand_usd = "/home/adityanair/assets/allegro_hand.usd"

    def load_arm_with_hand(name, pos):
        arm_path = f"/World/{name}/Franka"
        hand_path = f"/World/{name}/Hand"

        create_prim(arm_path, "Xform", usd_path=arm_usd)
        set_transform(arm_path, translation=pos)

        create_prim(hand_path, "Xform", usd_path=hand_usd)
        set_transform(hand_path, translation=(pos[0], pos[1], pos[2] + 0.4))

    load_arm_with_hand("Left", (-0.6, 0, 0))
    load_arm_with_hand("Right", (0.6, 0, 0))

    print("✅ Two arms and hands loaded")

run()

simulation_app.update()
simulation_app.close()