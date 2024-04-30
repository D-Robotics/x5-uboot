# Copyright (c) 2018, Linaro Limited
#
# SPDX-License-Identifier:  GPL-2.0+
#
# Android Verified Boot 2.0 Test

"""
This tests Android Verified Boot 2.0 support in U-boot:

For additional details about how to build proper vbmeta partition
check doc/android/avb2.rst

For configuration verification:
- Corrupt boot partition and check for failure
- Corrupt vbmeta partition and check for failure
"""

import pytest


@pytest.mark.buildconfigspec('ANDROID_AB')
def test_hb_ab(u_boot_console):
    """Run AVB 2.0 boot verification chain with avb subset of commands
    """

    response = u_boot_console.run_command('run ab_select_cmd')
    assert("ANDROID: Booting slot:" in response)

    response = u_boot_console.run_command('echo $bootslot')

    res = u_boot_console.run_command('ab_select test_get_slot mtd 0#misc')
    assert res == 'slot is :' + response

