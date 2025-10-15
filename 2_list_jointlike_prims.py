from pxr import Usd
import omni.usd

stage = omni.usd.get_context().get_stage()
JET_PATH = "/World/jetbot"   # robot root

print("\n--- Listing joint-like prims under", JET_PATH, "---")
for prim in Usd.PrimRange(stage.GetPrimAtPath(JET_PATH)):
    name = prim.GetName().lower()
    if "joint" in name or prim.GetTypeName().lower().startswith("joint") or "wheel" in name:
        print(f"{prim.GetPath()}  (type={prim.GetTypeName()})")



"""
expect output to be like this:

--- Listing joint-like prims under /World/jetbot ---
/World/jetbot/chassis/left_wheel_joint  (type=PhysicsRevoluteJoint)
/World/jetbot/chassis/right_wheel_joint  (type=PhysicsRevoluteJoint)
/World/jetbot/left_wheel  (type=Xform)
/World/jetbot/left_wheel/left_wheel  (type=Mesh)
/World/jetbot/left_wheel/left_wheel/wheel  (type=GeomSubset)
/World/jetbot/right_wheel  (type=Xform)
/World/jetbot/right_wheel/right_wheel  (type=Mesh)
/World/jetbot/right_wheel/right_wheel/wheel  (type=GeomSubset)
/World/jetbot/wheel_material  (type=Material)
"""
