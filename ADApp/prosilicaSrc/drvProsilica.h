#ifndef DRV_PROSILICA_H
#define DRV_PROSILICA_H

#ifdef __cplusplus
extern "C" {
#endif

int prosilicaSetup(int num_cameras);      /* Number of Prosilica cameras in system.  */

int prosilicaConfig(int camera,           /* Camera number */
                    char *ipAddr);        /* IP address of this camera.  If NULL then use first camera found */

#ifdef __cplusplus
}
#endif
#endif
