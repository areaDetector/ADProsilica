/* drvProsilica.h
 *
 * This is a driver for Prosilica cameras (GigE and CameraLink).
 *
 * Author: Mark Rivers
 *         University of Chicago
 *
 * Created:  March 20, 2008
 *
 */
#ifndef DRV_PROSILICA_H
#define DRV_PROSILICA_H

#ifdef __cplusplus
extern "C" {
#endif

extern ADDrvSet_t ADProsilica; 
int prosilicaSetup(int num_cameras);      /* Number of Prosilica cameras in system.  */
int prosilicaConfig(int camera,           /* Camera number */
                    char *ipAddr);        /* IP address of this camera.  If NULL then use first camera found */

#ifdef __cplusplus
}
#endif
#endif
