#!/usr/bin/env python3
import os, sys, pyheif
from PIL import Image

def image_convertor(src_folder, dst_folder, prefix="cube_"):
    os.makedirs(dst_folder, exist_ok=True)

    # 1) Find the highest existing index in dst_folder
    existing = [f for f in os.listdir(dst_folder)
                if f.startswith(prefix) and f.lower().endswith(".png")]
    nums = []
    for f in existing:
        try:
            # assume names like cube_01.png, cube_23.png, etc.
            num = int(os.path.splitext(f)[0].split('_')[-1])
            nums.append(num)
        except ValueError:
            pass
    # start from max+1, or 1 if none
    counter = max(nums, default=0) + 1

    # 2) Process HEIC files
    files = sorted(f for f in os.listdir(src_folder) if f.lower().endswith(".heic"))
    for file in files:
        src = os.path.join(src_folder, file)
        try:
            heif_file = pyheif.read(src)
            img = Image.frombytes(heif_file.mode, heif_file.size, heif_file.data,
                                  "raw", heif_file.mode, heif_file.stride)
        except Exception as e:
            print(f"❌ Error reading {src}: {e}")
            continue

        dst_name = f"{prefix}{counter:02d}.png"
        dst = os.path.join(dst_folder, dst_name)
        try:
            img.save(dst, "PNG")
            print(f"✅ Saved {dst}")
        except Exception as e:
            print(f"❌ Error saving {dst}: {e}")

        counter += 1

if __name__=='__main__':
    # your fixed source folder of HEICs:
    src_folder = "/home/megha/git/RS2/yolov8_detector_py/datasets/shapes/images/toconvert"
    # where you want your PNGs to go:
    dst_folder = "/home/megha/git/RS2/yolov8_detector_py/datasets/shapes/images/train"
    # filename prefix (e.g. cube_, cylinder_, cone_)
    prefix     = "cylinder_"
    image_convertor(src_folder, dst_folder, prefix)
