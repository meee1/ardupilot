#!/usr/bin/env python
# encoding: utf-8

"""
Waf tool for WolfMQTT build
"""

from waflib import Errors, Logs, Task, Utils
from waflib.TaskGen import after_method, before_method, feature

import os
import shutil
import sys
import re
import pickle

def configure(cfg):
    cfg.find_program('autoconf')
    env = cfg.env
    bldnode = cfg.bldnode.make_node(cfg.variant)
    def srcpath(path):
        return cfg.srcnode.make_node(path).abspath()

    def bldpath(path):
        return bldnode.make_node(path).abspath()
    env.wolfMQTT_ROOT =   srcpath('modules/wolfMQTT')
    env.wolfMQTT_BUILDDIR = bldpath('modules/wolfMQTT')
    if env.TOOLCHAIN == 'native':
        env.WOLFSSL_USER_HEADERS = cfg.path.find_node('libraries/AP_HAL_SITL/wolfssl/').abspath()
    elif env.TOOLCHAIN == 'arm-none-eabi':
        env.WOLFSSL_USER_HEADERS = cfg.path.find_node('libraries/AP_HAL_ChibiOS/hwdef/common/wolfssl/').abspath()
    env.EXTRA_CFLAGS = cfg.env.CPU_FLAGS + ['-fno-builtin']
    if cfg.env.DEBUG:
        env.EXTRA_CFLAGS += ['-g']
    try:
        cmd = "cd {0} && ./autogen.sh".format(env.wolfMQTT_ROOT)
        import subprocess
        ret = subprocess.call(cmd, shell=True)
    except Exception:
        cfg.fatal("Failed to setup autoconf in wolfMQTT")
    if ret != 0:
        cfg.fatal("Failed to setup autoconf in wolfMQTT ret=%d" % ret)

def build(bld):
    if bld.env.TOOLCHAIN == 'native':
        wolfMQTT_task = bld(
            #build libwolfMQTT.a from wolfMQTT sources
            rule="cd ${wolfMQTT_ROOT} && ./configure --prefix=${wolfMQTT_BUILDDIR} CFLAGS=\"${EXTRA_CFLAGS} -I${WOLFSSL_USER_HEADERS} -I../wolfssl -Wno-error=unused-parameter\" --disable-shared LDFLAGS=\"-L../wolfssl/src/.libs\" && make clean && make && make install",
            group='dynamic_sources',
            #source=wolfMQTT_source,
            target=bld.bldnode.find_or_declare('modules/wolfMQTT/lib/libwolfMQTT.a')
        )
    elif bld.env.TOOLCHAIN == 'arm-none-eabi':
        wolfMQTT_task = bld(
            #build libwolfMQTT.a from wolfMQTT sources
            rule="cd ${wolfMQTT_ROOT} && ./configure --host=arm-none-eabi CC=arm-none-eabi-gcc AR=arm-none-eabi-ar STRIP=arm-none-eabi-strip RANLIB=arm-none-eabi-ranlib --prefix=${wolfMQTT_BUILDDIR} CFLAGS=\"--specs=nosys.specs -mthumb ${EXTRA_CFLAGS} -I${WOLFSSL_USER_HEADERS} -I../wolfssl -Wno-error=unused-parameter\" --disable-filesystem --disable-fastmath --disable-shared --enable-sha --enable-keygen --enable-rsa --disable-oldnames && make clean && make && make install",
            group='dynamic_sources',
            #source=wolfMQTT_source,
            target=bld.bldnode.find_or_declare('modules/wolfMQTT/lib/libwolfMQTT.a')
        )
    wolfMQTT_task.name = "wolfMQTT_lib"
    bld.env.LIB += ['wolfMQTT']
    bld.env.LIBPATH += ['modules/wolfMQTT/lib']
    bld.env.INCLUDES += ['modules/wolfMQTT/include']
