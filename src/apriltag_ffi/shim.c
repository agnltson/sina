#include "shim.h"

#include <string.h>
#include <stdlib.h>

#include "apriltag.h"
#include "apriltag_pose.h"
#include "common/matd.h"

#include "tag16h5.h"
#include "tag25h9.h"
#include "tag36h11.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"

struct shim_detector {
    apriltag_detector_t *td;
    apriltag_family_t *family;
    char family_name[32];
};

struct shim_detections {
    zarray_t *detections; /* zarray_t<apriltag_detection_t*> */
};

static apriltag_family_t *create_family(const char *name) {
    if (strcmp(name, "tag16h5") == 0) return tag16h5_create();
    if (strcmp(name, "tag25h9") == 0) return tag25h9_create();
    if (strcmp(name, "tag36h11") == 0) return tag36h11_create();
    if (strcmp(name, "tagcircle21h7") == 0) return tagCircle21h7_create();
    if (strcmp(name, "tagcircle49h12") == 0) return tagCircle49h12_create();
    if (strcmp(name, "tagcustom48h12") == 0) return tagCustom48h12_create();
    if (strcmp(name, "tagstandard41h12") == 0) return tagStandard41h12_create();
    if (strcmp(name, "tagstandard52h13") == 0) return tagStandard52h13_create();
    return NULL;
}

static void destroy_family(const char *name, apriltag_family_t *fam) {
    if (!fam) return;
    if (strcmp(name, "tag16h5") == 0) { tag16h5_destroy(fam); return; }
    if (strcmp(name, "tag25h9") == 0) { tag25h9_destroy(fam); return; }
    if (strcmp(name, "tag36h11") == 0) { tag36h11_destroy(fam); return; }
    if (strcmp(name, "tagcircle21h7") == 0) { tagCircle21h7_destroy(fam); return; }
    if (strcmp(name, "tagcircle49h12") == 0) { tagCircle49h12_destroy(fam); return; }
    if (strcmp(name, "tagcustom48h12") == 0) { tagCustom48h12_destroy(fam); return; }
    if (strcmp(name, "tagstandard41h12") == 0) { tagStandard41h12_destroy(fam); return; }
    if (strcmp(name, "tagstandard52h13") == 0) { tagStandard52h13_destroy(fam); return; }
}

shim_detector_t *shim_detector_create(
    const char *family_name,
    int nthreads,
    float quad_decimate,
    float quad_sigma,
    int refine_edges
) {
    apriltag_family_t *fam = create_family(family_name);
    if (!fam) return NULL;

    apriltag_detector_t *td = apriltag_detector_create();
    if (!td) {
        destroy_family(family_name, fam);
        return NULL;
    }
    apriltag_detector_add_family_bits(td, fam, 1);
    td->nthreads = nthreads;
    td->quad_decimate = quad_decimate;
    td->quad_sigma = quad_sigma;
    td->refine_edges = refine_edges;

    shim_detector_t *shim = (shim_detector_t *)malloc(sizeof(shim_detector_t));
    if (!shim) {
        apriltag_detector_destroy(td);
        destroy_family(family_name, fam);
        return NULL;
    }
    shim->td = td;
    shim->family = fam;
    strncpy(shim->family_name, family_name, sizeof(shim->family_name) - 1);
    shim->family_name[sizeof(shim->family_name) - 1] = '\0';
    return shim;
}

void shim_detector_destroy(shim_detector_t *det) {
    if (!det) return;
    apriltag_detector_destroy(det->td);
    destroy_family(det->family_name, det->family);
    free(det);
}

shim_detections_t *shim_detect(
    shim_detector_t *det,
    const uint8_t *buf,
    int32_t width,
    int32_t height,
    int32_t stride
) {
    if (!det) return NULL;

    /* apriltag veut gérer lui-même le stride/alignement interne de son
     * buffer (optimisé SIMD), donc on alloue via image_u8_create et on
     * copie ligne par ligne depuis notre buffer d'entrée. C'est le point
     * qui causait des soucis d'alignement côté crate Rust historique. */
    image_u8_t *im = image_u8_create((unsigned int)width, (unsigned int)height);
    if (!im) return NULL;

    for (int32_t row = 0; row < height; row++) {
        memcpy(im->buf + (size_t)row * (size_t)im->stride,
               buf + (size_t)row * (size_t)stride,
               (size_t)width);
    }

    zarray_t *detections = apriltag_detector_detect(det->td, im);
    image_u8_destroy(im);

    shim_detections_t *out = (shim_detections_t *)malloc(sizeof(shim_detections_t));
    if (!out) {
        if (detections) apriltag_detections_destroy(detections);
        return NULL;
    }
    out->detections = detections;
    return out;
}

int32_t shim_detections_count(const shim_detections_t *dets) {
    if (!dets || !dets->detections) return 0;
    return (int32_t)zarray_size(dets->detections);
}

int32_t shim_detection_id(const shim_detections_t *dets, int32_t idx) {
    apriltag_detection_t *d;
    zarray_get(dets->detections, idx, &d);
    return d->id;
}

void shim_detection_center(const shim_detections_t *dets, int32_t idx, double out_center[2]) {
    apriltag_detection_t *d;
    zarray_get(dets->detections, idx, &d);
    out_center[0] = d->c[0];
    out_center[1] = d->c[1];
}

void shim_detections_destroy(shim_detections_t *dets) {
    if (!dets) return;
    if (dets->detections) {
        apriltag_detections_destroy(dets->detections);
    }
    free(dets);
}

static void matd_to_flat3x3(const matd_t *m, double out[9]) {
    for (unsigned int r = 0; r < 3; r++) {
        for (unsigned int c = 0; c < 3; c++) {
            out[r * 3 + c] = matd_get(m, r, c);
        }
    }
}

static void matd_to_flat3(const matd_t *m, double out[3]) {
    for (unsigned int r = 0; r < 3; r++) {
        out[r] = matd_get(m, r, 0);
    }
}

int shim_estimate_pose_orthogonal(
    const shim_detections_t *dets,
    int32_t idx,
    double tagsize_m,
    double fx, double fy, double cx, double cy,
    int n_iters,
    double *out_err1, double out_R1[9], double out_t1[3],
    double *out_err2, double out_R2[9], double out_t2[3]
) {
    if (!dets || !dets->detections) return -1;
    apriltag_detection_t *d;
    zarray_get(dets->detections, idx, &d);

    apriltag_detection_info_t info;
    info.det = d;
    info.tagsize = tagsize_m;
    info.fx = fx;
    info.fy = fy;
    info.cx = cx;
    info.cy = cy;

    apriltag_pose_t pose1;
    apriltag_pose_t pose2;
    memset(&pose1, 0, sizeof(pose1));
    memset(&pose2, 0, sizeof(pose2));

    estimate_tag_pose_orthogonal_iteration(&info, out_err1, &pose1, out_err2, &pose2, n_iters);

    int result = 0;

    if (!pose1.R || !pose1.t || pose1.R->nrows != 3 || pose1.R->ncols != 3
        || pose1.t->nrows != 3 || pose1.t->ncols != 1) {
        result = -2;
    } else {
        matd_to_flat3x3(pose1.R, out_R1);
        matd_to_flat3(pose1.t, out_t1);

        if (pose2.R && pose2.t && pose2.R->nrows == 3 && pose2.R->ncols == 3
            && pose2.t->nrows == 3 && pose2.t->ncols == 1) {
            matd_to_flat3x3(pose2.R, out_R2);
            matd_to_flat3(pose2.t, out_t2);
        } else {
            *out_err2 = -1.0; /* pas de deuxième solution */
        }
    }

    matd_destroy(pose1.R);
    matd_destroy(pose1.t);
    matd_destroy(pose2.R);
    matd_destroy(pose2.t);

    return result;
}
