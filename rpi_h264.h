/*
 * rpi_h264.h
 *
 *  Created on: Nov 1, 2021
 *      Author: tgjuranec
 */

#ifndef RPI_H264_H_
#define RPI_H264_H_

#ifdef 	RPI_H264_HDEC

/*
Copyright (c) 2018 Raspberry Pi (Trading) Ltd

Decodes a JPEG image into a memory buffer.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

//#include <interface/vcsm/user-vcsm.h>
#include "bcm_host.h"
#include "interface/mmal/mmal.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/mmal_queue.h"
#include "interface/vcos/vcos.h"

#define MAX_BUFFERS 2

static inline int warn(const char *file, int line, const char *fmt, ...)
{
   int errsv = errno;
   va_list va;
   va_start(va, fmt);
   fprintf(stderr, "WARN(%s:%d): ", file, line);
   vfprintf(stderr, fmt, va);
   va_end(va);
   errno = errsv;
   return 1;
}

#define CHECK_CONDITION(cond, ...) \
do { \
   if (cond) { \
      int errsv = errno; \
      fprintf(stderr, "ERROR(%s:%d) : ", \
         __FILE__, __LINE__); \
      errno = errsv; \
      fprintf(stderr,  __VA_ARGS__); \
      abort(); \
   } \
} while(0)
#define WARN_ON(cond, ...) \
   ((cond) ? warn(__FILE__, __LINE__, __VA_ARGS__) : 0)
#define ERRSTR strerror(errno)
#define CHECK_STATUS(status, ...) WARN_ON(status != MMAL_SUCCESS, __VA_ARGS__); \
   if (status != MMAL_SUCCESS) goto error;

static FILE *source_file;

/* Macros abstracting the I/O, just to make the example code clearer */
#define SOURCE_OPEN(uri) \
   source_file = fopen(uri, "rb"); if (!source_file) goto error;
#define SOURCE_READ_CODEC_CONFIG_DATA(bytes, size) \
   size = fread(bytes, 1, size, source_file); rewind(source_file)
#define SOURCE_READ_DATA_INTO_BUFFER(a) \
   a->length = fread(a->data, 1, a->alloc_size - 128, source_file); \
   a->offset = 0
#define SOURCE_CLOSE() \
   if (source_file) fclose(source_file)


#endif


#endif /* RPI_H264_H_ */
