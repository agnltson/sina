import argparse
import json
from pathlib import Path

def generate_mp4(vrs_path: str, mp4_out: str) -> None:
    from projectaria_tools.utils.vrs_to_mp4_utils import convert_vrs_to_mp4

    print(f"Extracting MP4 from {vrs_path}")
    log_folder = str(Path(mp4_out).parent)
    convert_vrs_to_mp4(vrs_path, mp4_out, log_folder, 1)
    print(f"MP4 extracted: {mp4_out}")

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
            "--vrs",
            required=True,
            help="Path to .vrs source. Not needed if --skip-generation is set"
            )
    parser.add_argument(
            "--mp4_out",
            required=True,
            help="Path of the output mp4"
            )
    args = parser.parse_args()

    if not args.vrs:
        parser.error("--vrs required")
    generate_mp4(args.vrs, args.mp4_out)

if __name__ == "__main__":
    main()
