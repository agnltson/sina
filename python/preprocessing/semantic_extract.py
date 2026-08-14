def main():
    import sys
    import numpy as np
    from scipy.spatial.transform import Rotation

    import plotly.graph_objects as go

    from src.data.language_sequence import LanguageSequence
    from src.data.point_cloud import PointCloud
    from src.networks.scenescript_model import SceneScriptWrapper

    print("All imports done")

    ckpt_path = "/home/knette/stage/scenescript_model_non_manhattan_class_agnostic_model.ckpt"
    model_wrapper = SceneScriptWrapper.load_from_checkpoint(ckpt_path).cuda()

    if len(sys.argv) < 2:
        print("Missing point cloud path")
        return;


    point_cloud_path = sys.argv[1]
    point_cloud_obj = PointCloud.load_from_file(point_cloud_path)

    lang_seq = model_wrapper.run_inference(
        point_cloud_obj.points,
        nucleus_sampling_thresh=0.05,
        verbose=True,
    )

    output_file_path = "semantic.txt"
    with open(output_file_path, "w") as f:
        f.write(lang_seq.generate_language_string())
        print(f"Semantic wrote at {output_file_path}")

if __name__ == "__main__":
    main()
