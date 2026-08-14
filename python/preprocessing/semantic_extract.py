def main():
    import argparse
    import numpy as np
    from scipy.spatial.transform import Rotation
    import plotly.graph_objects as go
    from src.data.language_sequence import LanguageSequence
    from src.data.point_cloud import PointCloud
    from src.networks.scenescript_model import SceneScriptWrapper

    parser = argparse.ArgumentParser(
        description="Run SceneScript inference on a point cloud."
    )
    parser.add_argument(
        "point_cloud_path",
        type=str,
        help="Path to the point cloud file to process.",
    )
    parser.add_argument(
        "--ckpt-path",
        type=str,
        help="Path to the model checkpoint (scenescript_model_non_manhattan_class_agnostic_model.ckpt) file.",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="semantic.txt",
        help="Path to the output file (default: semantic.txt).",
    )
    args = parser.parse_args()

    print("All imports done")
    model_wrapper = SceneScriptWrapper.load_from_checkpoint(args.ckpt_path).cuda()

    point_cloud_obj = PointCloud.load_from_file(args.point_cloud_path)
    lang_seq = model_wrapper.run_inference(
        point_cloud_obj.points,
        nucleus_sampling_thresh=0.05,
        verbose=True,
    )

    with open(args.output, "w") as f:
        f.write(lang_seq.generate_language_string())
        print(f"Semantic wrote at {args.output}")


if __name__ == "__main__":
    main()
