from pxr import Usd, UsdGeom
import omni.usd

stage = omni.usd.get_context().get_stage()
print("Stage root:", stage.GetRootLayer().identifier)

# Find Jetson / Jetbot prim
jet_path = None
for prim in Usd.PrimRange(stage.GetPseudoRoot()):
    name = prim.GetName().lower()
    if 'jet' in name:
        print("Possible Jet prim:", prim.GetPath())
        jet_path = prim.GetPath()

if not jet_path:
    raise RuntimeError("No prim containing 'jet' found!")

print("Using jet path:", jet_path)
jet = stage.GetPrimAtPath(jet_path)
if not jet:
    raise RuntimeError(f"Prim not found: {jet_path}")

# Recursive prim dump
def dump_prim(prim, depth=0, max_depth=3):
    indent = "  " * depth
    print(f"{indent}{prim.GetPath()}  (type={prim.GetTypeName()})")
    for attr in prim.GetAttributes():
        try:
            print(f"{indent}  - {attr.GetName()} = {attr.Get()}")
        except Exception:
            print(f"{indent}  - {attr.GetName()} = <unreadable>")
    for child in prim.GetChildren():
        dump_prim(child, depth+1, max_depth)

dump_prim(jet)

# List all camera prims
print("\nCameras found:")
for prim in Usd.PrimRange(stage.GetPseudoRoot()):
    if prim.IsA(UsdGeom.Camera):
        print(" -", prim.GetPath())

# Example: check one camera (replace with actual path)
camera_path = "/World/jetbot/chassis/rgb_camera/jetbot_camera"
camera_prim = stage.GetPrimAtPath(camera_path)
print("\nFound camera prim:", camera_prim)




"""
Expect output to be like this:

Stage root: anon:0x75a2d8811060:World0.usd
Possible Jet prim: /World/jetbot
Possible Jet prim: /World/jetbot/chassis/rgb_camera/jetbot_camera
Using jet path: /World/jetbot/chassis/rgb_camera/jetbot_camera
/World/jetbot/chassis/rgb_camera/jetbot_camera  (type=Camera)
  - clippingPlanes = []
  - clippingRange = (0.1, 1000000)
  - exposure = 0.0
  - focalLength = 0.75
  - focusDistance = 0.0
  - fStop = 0.0
  - horizontalAperture = 2.3499999046325684
  - horizontalApertureOffset = 0.0
  - omni:kit:centerOfInterest = (0, 0, -100)
  - projection = perspective
  - purpose = default
  - shutter:close = 0.0
  - shutter:open = 0.0
  - stereoRole = mono
  - verticalAperture = 2.3499999046325684
  - verticalApertureOffset = 0.0
  - visibility = inherited
  - xformOp:rotateZYX = (90, -90, 0)
  - xformOp:scale = (1, 0.9999989, 1.000001)
  - xformOp:translate = (0.024325408, -2.1756853e-14, -3.5110803e-14)
  - xformOpOrder = [xformOp:translate, xformOp:rotateZYX, xformOp:scale]

Cameras found:
 - /World/jetbot/chassis/rgb_camera/jetbot_camera
 - /OmniverseKit_Persp
 - /OmniverseKit_Front
 - /OmniverseKit_Top
 - /OmniverseKit_Right

Found camera prim: Usd.Prim(</World/jetbot/chassis/rgb_camera/jetbot_camera>)
"""
