use std::path::PathBuf;

fn main() {
    let vendor_dir = PathBuf::from("vendor/apriltag");
    if !vendor_dir.join("apriltag.h").exists() {
        panic!(
            "Cannot find vendor/apriltag.\n\
             Run `git submodule update --init --recursive`\n\
             (or clone https://github.com/AprilRobotics/apriltag inside vendor/apriltag)."
        );
    }

    let mut build = cc::Build::new();
    build
        .include(&vendor_dir)
        .include("src/apriltag_ffi")
        .flag_if_supported("-std=gnu17")
        .warnings(false);

    let sources = [
        "apriltag.c",
        "apriltag_pose.c",
        "apriltag_quad_thresh.c",
        "tag16h5.c",
        "tag25h9.c",
        "tag36h11.c",
        "tagCircle21h7.c",
        "tagCircle49h12.c",
        "tagCustom48h12.c",
        "tagStandard41h12.c",
        "tagStandard52h13.c",
        "common/g2d.c",
        "common/getopt.c",
        "common/homography.c",
        "common/image_u8.c",
        "common/image_u8_parallel.c",
        "common/image_u8x3.c",
        "common/image_u8x4.c",
        "common/matd.c",
        "common/pjpeg.c",
        "common/pjpeg-idct.c",
        "common/pnm.c",
        "common/string_util.c",
        "common/svd22.c",
        "common/time_util.c",
        "common/unionfind.c",
        "common/workerpool.c",
        "common/zarray.c",
        "common/zhash.c",
        "common/zmaxheap.c",
    ];

    for src in sources {
        let path = vendor_dir.join(src);
        if !path.exists() {
            panic!(
                "Cannot find expected apriltag file: {}. \
                 Submodule vendor/apriltag might be missing files or \
                 original repo might have change.",
                path.display()
            );
        }
        build.file(path);
    }

    build.file("src/apriltag_ffi/shim.c");
    build.compile("apriltag_shim");

    println!("cargo:rerun-if-changed=src/apriltag_ffi/shim.c");
    println!("cargo:rerun-if-changed=src/apriltag_ffi/shim.h");
    println!("cargo:rerun-if-changed=vendor/apriltag");

    #[cfg(unix)]
    {
        println!("cargo:rustc-link-lib=pthread");
        println!("cargo:rustc-link-lib=m");
    }
}
