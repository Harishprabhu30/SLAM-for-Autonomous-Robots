from pxr import UsdPhysics
import omni.usd

stage = omni.usd.get_context().get_stage()
jet_prim = stage.GetPrimAtPath("/World/jetbot")

print("ArticulationRootAPI applied:", jet_prim.HasAPI(UsdPhysics.ArticulationRootAPI))


"""
Expect output to be:

✅ If it prints True, your robot is already an articulation root → good to go.
❌ If it prints False, do this once manually:

Right-click the prim /World/jetbot in the Stage tree.

Add → Physics → Articulation Root.

Then rerun the snippet to confirm it shows True.
"""
