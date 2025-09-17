#!/usr/bin/env python

import os
import sys
import shutil
from collections import OrderedDict

"""
Multiple dtb package

The board is what you defined in BoardConfig.mk like PRODUCT_KERNEL_BOARD.
Such as: PRODUCT_KERNEL_BOARD := rk3566_ht_eink_public&#gpio2b4=0#gpio2b0=0 rk3566_ht_eink_public1&#gpio2b4=1#gpio2b0=1

"""
def packMultiDtbs(args):
    print "Will repack the multiple dtbs in to one. "

    # 1.get the boards
    board_list = args.split()

    if (len(board_list) < 2):
        print "The board config too few!!"
        sys.exit(1)

    # 2.get the board config
    DTBS = OrderedDict()
    for board in board_list:
        if '@' not in board:
            print "The board config format error!!"
            sys.exit(1)
        board = board.replace("@", "#")
        dts_list = board.split('&')
        DTBS[dts_list[0]] = dts_list[1]

    # 3.set default dtb, and rename the dtb with copy
    target_dtb_list = ''
    default_dtb = True

    for dtb, value in DTBS.items():
        if default_dtb:
            ori_file = 'arch/arm64/boot/dts/rockchip/' + dtb + '.dtb'
            shutil.copyfile(ori_file, "rk-kernel.dtb")
            target_dtb_list += 'rk-kernel.dtb '
            default_dtb = False
        new_file = dtb + value + '.dtb'
        ori_file = 'arch/arm64/boot/dts/rockchip/' + dtb + '.dtb'
        shutil.copyfile(ori_file, new_file)
        target_dtb_list += ' ' + new_file

    # 4.repack the all dtb to one
    print "Repack dtbs: ", target_dtb_list
    os.system('scripts/resource_tool --image=resource.img ' + target_dtb_list)
    os.system('rm ' + target_dtb_list)
