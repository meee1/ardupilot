/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _ASM_ALPHA_IOCTLS_H
#define _ASM_ALPHA_IOCTLS_H

//#include <bits/ioctl.h>

#define TCGETS2		_IOR('T', 0x2A, struct termios2)
#define TCSETS2		_IOW('T', 0x2B, struct termios2)
#define TCSETSW2	_IOW('T', 0x2C, struct termios2)
#define TCSETSF2	_IOW('T', 0x2D, struct termios2)

#endif /* _ASM_ALPHA_IOCTLS_H */