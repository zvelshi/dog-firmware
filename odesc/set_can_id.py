import argparse, time, odrive
from odrive.enums import *

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--axis",  type=int, default=0, choices=[0,1])
    ap.add_argument("--node_id", type=int, required=True, help="0‑63")
    ap.add_argument("--baud", type=int, help="optionally set new CAN bitrate")
    args = ap.parse_args()

    print("Looking for ODrive")
    odrv = odrive.find_any()
    ax   = odrv.axis0 if args.axis == 0 else odrv.axis1

    print(f"Current node‑id: {ax.config.can_node_id}")
    if not (0 <= args.node_id <= 63):
        raise ValueError("node‑id must be 0‑63")

    ax.config.can_node_id = args.node_id          # path <axis>.config.can.node_id :contentReference[oaicite:1]{index=1}
    if args.baud:
        odrv.can.config.baud_rate = args.baud     # optional

    print("Save and reboot")
    odrv.save_configuration()
    odrv.reboot()                                 # drive comes back with new CAN‑ID

if __name__ == "__main__":
    main()