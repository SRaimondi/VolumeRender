/*
 * dds_loader.h
 *
 *  Created on: Mar 23, 2018
 *      Author: simon
 */

#ifndef DDS_LOADER_H_
#define DDS_LOADER_H_

#include "Common.h"

// Code taken from V^3 library to load PVM file

unsigned char *readPVMvolume(const char *filename,
							 unsigned int *width, unsigned int *height,
							 unsigned int *depth, unsigned int *components = nullptr,
							 float *scalex = nullptr, float *scaley = nullptr, float *scalez = nullptr,
							 unsigned char **description = nullptr,
							 unsigned char **courtesy = nullptr,
							 unsigned char **parameter = nullptr,
							 unsigned char **comment = nullptr);

unsigned char *readDDSfile(const char *filename, unsigned int *bytes);

unsigned char *readRAWfile(const char *filename, unsigned int *bytes);

#endif /* DDS_LOADER_H_ */
