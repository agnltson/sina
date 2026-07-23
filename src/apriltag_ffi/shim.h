#ifndef APRILTAG_SHIM_H
#define APRILTAG_SHIM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Types opaques : côté Rust on ne connaît jamais le layout interne de la
 * lib apriltag, seulement des pointeurs. Toute la connaissance du layout
 * (apriltag_detector_t, apriltag_detection_t, matd_t, ...) reste ici, dans
 * du C compilé directement contre les vrais headers vendorés. */
typedef struct shim_detector shim_detector_t;
typedef struct shim_detections shim_detections_t;

/* Crée un détecteur pour la famille `family_name` parmi :
 * tag16h5, tag25h9, tag36h11, tagcircle21h7, tagcircle49h12,
 * tagcustom48h12, tagstandard41h12, tagstandard52h13.
 * Retourne NULL si la famille est inconnue ou en cas d'échec d'allocation. */
shim_detector_t *shim_detector_create(
    const char *family_name,
    int nthreads,
    float quad_decimate,
    float quad_sigma,
    int refine_edges
);

void shim_detector_destroy(shim_detector_t *det);

/* Lance la détection sur un buffer 8 bits niveaux de gris, row-major,
 * `stride` octets par ligne (stride >= width). Retourne NULL en cas
 * d'échec d'allocation interne. Le résultat doit être libéré avec
 * shim_detections_destroy. */
shim_detections_t *shim_detect(
    shim_detector_t *det,
    const uint8_t *buf,
    int32_t width,
    int32_t height,
    int32_t stride
);

int32_t shim_detections_count(const shim_detections_t *dets);
int32_t shim_detection_id(const shim_detections_t *dets, int32_t idx);

/* Remplit out_center[2] = {x, y} avec le centre du tag en pixels. */
void shim_detection_center(const shim_detections_t *dets, int32_t idx, double out_center[2]);

void shim_detections_destroy(shim_detections_t *dets);

/* Estimation de pose par itération orthogonale pour la détection `idx`.
 * out_R1/out_R2 : matrices de rotation 3x3 row-major (9 doubles).
 * out_t1/out_t2 : vecteurs de translation (3 doubles).
 * Si aucune deuxième solution locale n'est trouvée, *out_err2 vaut -1.0.
 * Retourne 0 en cas de succès, une valeur négative en cas d'échec. */
int shim_estimate_pose_orthogonal(
    const shim_detections_t *dets,
    int32_t idx,
    double tagsize_m,
    double fx, double fy, double cx, double cy,
    int n_iters,
    double *out_err1, double out_R1[9], double out_t1[3],
    double *out_err2, double out_R2[9], double out_t2[3]
);

#ifdef __cplusplus
}
#endif

#endif /* APRILTAG_SHIM_H */
