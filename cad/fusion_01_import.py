# Asama 1: 23 parcayi ayri bilesen olarak ice aktar + 12 sabit parcayi Rigid Group yap.
# Parcalar mutlak konumda uretildigi icin occurrence donusumu birim matris kalir;
# "orijine gore yerlestir" ile gelen her parca kendiliginde dogru oturur.
#
# KULLANIM (Fusion 360 > Utilities > Scripts and Add-Ins > Scripts > + > bu dosya):
#   1. Bos bir Design dosyasi acin.
#   2. Scripti calistirin. Yol otomatik bulunmazsa asagidaki BASE'i doldurun.
#   3. Bittiginde 23 bilesen ve "base_link" adli Rigid Group olusmus olur.
#
# Sonraki asama: 11 hareketli parca icin revolute eklemler -- eklem tablosu ve
# limitler MONTAJ.md'de. KUYRUK TILT LIMITI 0...20 DERECE, kanatlarinki gibi
# 90 degil; 90 yazilirsa disk kuyruk cubugunun icinden gecer.
import os
import adsk.core
import adsk.fusion
import traceback

# Parca klasoru. Bos birakilirsa asagidaki adaylar sirayla denenir; hicbiri
# tutmazsa script durur ve nereye bakmasi gerektigini soyler.
BASE = ""

CANDIDATES = [
    os.environ.get("TILTROTOR_CAD_PARTS", ""),
    os.path.join(os.path.expanduser("~"), "Documents", "tiltrotor_cad", "step", "parts"),
    os.path.join(os.path.expanduser("~"), "tiltrotor_project_updates", "cad", "step", "parts"),
    os.path.join(os.path.expanduser("~"), "SOLOUAV", "cad", "step", "parts"),
    # Wine kurulumundan kalan eski yol (Linux donemi, artik kullanilmiyor)
    r"C:\users\umran\Documents\tiltrotor_cad\step\parts",
]

# Parca listesi KLASORDEN turetilir, elle yazilmaz: build_mechanism.py
# mekanizma parcalarini ekledikten sonra sayi 23'ten 56'ya cikti ve sabit
# liste sessizce eksik kalmisti. Hareketli olanlar ad kalibiyla ayrilir;
# geri kalan her sey base_link Rigid Group'una girer.
MOVING_PREFIX = ("motor_", "rotor_", "elevon_", "elevator_",
                 "tilt_cradle_", "tilt_crank_", "horn_", "pushrod_")
MOVING_EXACT = ("rudder",)


def siniflandir(names):
    """(STATIC, MOVING) — hareketli = eklemle donen ya da onunla birlikte giden."""
    moving = [n for n in names
              if n.startswith(MOVING_PREFIX) or n in MOVING_EXACT]
    static = [n for n in names if n not in moving]
    return static, moving


def find_base():
    """Parca klasorunu bul: once BASE, sonra CANDIDATES."""
    for path in ([BASE] if BASE else []) + CANDIDATES:
        if path and os.path.isdir(path):
            return path
    return None


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        base = find_base()
        if base is None:
            ui.messageBox(
                "STEP parca klasoru bulunamadi.\n\n"
                "Bu dosyanin basindaki BASE degiskenine parts klasorunun tam\n"
                "yolunu yazin, ornegin:\n"
                "  BASE = r'C:\\Users\\umran\\Documents\\SOLOUAV\\cad\\step\\parts'\n\n"
                "Denenen yollar:\n  " + "\n  ".join(p for p in CANDIDATES if p))
            return

        names = sorted(f[:-5] for f in os.listdir(base) if f.endswith(".step"))
        if not names:
            ui.messageBox("Klasorde hic .step yok:\n{}\n\n"
                          "Once cad/build_tiltrotor_cad.py, sonra "
                          "cad/build_mechanism.py calistirin.".format(base))
            return
        STATIC, MOVING = siniflandir(names)
        PARTS = STATIC + MOVING

        design = adsk.fusion.Design.cast(
            app.activeDocument.products.itemByProductType("DesignProductType"))
        if design is None:
            ui.messageBox("Aktif bir Design dosyasi yok. Bos bir Design acip tekrar calistirin.")
            return

        root = design.rootComponent
        var_olan = [n for n in PARTS
                    if any(o.component.name == n for o in root.occurrences)]
        if var_olan:
            ui.messageBox("Bu dosyada ayni adli bilesenler zaten var ({}...).\n\n"
                          "Ikinci kez calistirmak kopya uretir. Bos bir Design acin."
                          .format(", ".join(var_olan[:3])))
            return

        # Zaman cizelgesini kapat: onlarca ice aktarma cok fazla ozellik uretir
        design.designType = adsk.fusion.DesignTypes.DirectDesignType

        im = app.importManager
        made = {}
        for name in PARTS:
            occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            occ.component.name = name
            opts = im.createSTEPImportOptions(os.path.join(base, name + ".step"))
            opts.isViewFit = False
            im.importToTarget(opts, occ.component)
            made[name] = occ

        # Sabit govdeleri tek Rigid Group yap
        col = adsk.core.ObjectCollection.create()
        for n in STATIC:
            col.add(made[n])
        root.rigidGroups.add(col, True).name = "base_link"

        ui.messageBox("TAMAM.\n\nKlasor: {}\nBilesen: {}\n"
                      "base_link Rigid Group: {} sabit parca\n"
                      "Eklem bekleyen hareketli parca: {}\n\n"
                      "Sonraki adim: MONTAJ.md eklem tablosu.\n"
                      "Kuyruk tilt limiti 0...20 derece (90 DEGIL)."
                      .format(base, root.occurrences.count, len(STATIC), len(MOVING)))

    except:  # noqa: E722 -- Fusion script idiyomu
        if ui:
            ui.messageBox("Script hata verdi:\n{}".format(traceback.format_exc()))
        else:
            raise


# Fusion, Scripts panelinden calistirildiginda bu modulu import edip run()'i
# kendisi cagirir (o sirada __name__ modul adidir, asagidaki kosul tutmaz).
# Kopru uzerinden (fusion_execute) calistirildiginda ise kod duz exec edilir ve
# run()'i cagiran kimse olmaz -- iki yolu da desteklemek icin:
if globals().get("__name__") in (None, "__main__"):
    run(None)
