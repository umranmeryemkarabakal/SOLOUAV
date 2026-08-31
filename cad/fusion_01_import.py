# Asama 1: 23 parcayi ayri bilesen olarak ice aktar + 12 sabit parcayi Rigid Group yap.
# Parcalar mutlak konumda uretildigi icin occurrence donusumu birim matris kalir.
import adsk.core, adsk.fusion, traceback

BASE = r"C:\users\umran\Documents\tiltrotor_cad\step\parts"

STATIC = ["wing", "winglet_left", "winglet_right", "tail_boom", "tailplane",
          "tailplane_strut", "vertical_stabiliser", "pylon_left", "pylon_right",
          "leg_front_left", "leg_front_right", "leg_tail"]
MOVING = ["motor_0_right", "motor_1_left", "motor_2_tail",
          "rotor_0_right", "rotor_1_left", "rotor_2_tail",
          "elevon_left", "elevon_right",
          "elevator_left", "elevator_right", "rudder"]

app = adsk.core.Application.get()
design = adsk.fusion.Design.cast(
    app.activeDocument.products.itemByProductType("DesignProductType"))
root = design.rootComponent
im = app.importManager

# Zaman cizelgesini kapat: 23 ice aktarma cok fazla ozellik uretir
design.designType = adsk.fusion.DesignTypes.DirectDesignType

made = {}
for name in STATIC + MOVING:
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    opts = im.createSTEPImportOptions(BASE + "\\" + name + ".step")
    opts.isViewFit = False
    im.importToTarget(opts, occ.component)
    made[name] = occ
    print("ice aktarildi:", name)

# Sabit govdeleri tek Rigid Group yap
col = adsk.core.ObjectCollection.create()
for n in STATIC:
    col.add(made[n])
root.rigidGroups.add(col, True).name = "base_link"
print("Rigid Group 'base_link' olusturuldu:", len(STATIC), "parca")
print("TAMAM. Toplam bilesen:", root.occurrences.count)
