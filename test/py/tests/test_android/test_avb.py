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
import u_boot_utils as util

# defauld mmc id
mmc_dev = 0

@pytest.mark.buildconfigspec('cmd_avb')
@pytest.mark.buildconfigspec('cmd_mmc')
def test_avb_verify(u_boot_console):
    """Run AVB 2.0 boot verification chain with avb subset of commands
    """

    success_str = "Verification passed successfully"

    response = u_boot_console.run_command('avb init mmc %s' % str(mmc_dev))
    assert response == ''
    response = u_boot_console.run_command('run ab_select_cmd')
    assert("ANDROID: Booting slot:" in response)
    response = u_boot_console.run_command('avb verify $slot_suffix')
    assert(success_str in response)


@pytest.mark.buildconfigspec('cmd_avb')
@pytest.mark.buildconfigspec('cmd_mmc')
def test_avb_mmc_uuid(u_boot_console):
    """Check if 'avb get_uuid' works, compare results with
    'part list mmc 1' output
    """

    response = u_boot_console.run_command('avb init mmc %s' % str(mmc_dev))
    assert response == ''

    response = u_boot_console.run_command('mmc rescan; mmc dev %s' %
                                          str(mmc_dev))
    assert('is current device' in response)

    part_lines = u_boot_console.run_command('mmc part').splitlines()
    part_list = {}
    cur_partname = ''

    for line in part_lines:
        if '"' in line:
            start_pt = line.find('"')
            end_pt = line.find('"', start_pt + 1)
            cur_partname = line[start_pt + 1: end_pt]

        if 'guid:' in line:
            guid_to_check = line.split('guid:\t')
            part_list[cur_partname] = guid_to_check[1]

    # lets check all guids with avb get_guid
    for part, guid in part_list.items():
        avb_guid_resp = u_boot_console.run_command('avb get_uuid %s' % part)
        assert guid == avb_guid_resp.split('UUID: ')[1]


@pytest.mark.buildconfigspec('cmd_avb')
def test_avb_read_rb(u_boot_console):
    """Test reading rollback indexes
    """

    response = u_boot_console.run_command('avb init mmc %s' % str(mmc_dev))
    assert response == ''

    response = u_boot_console.run_command('avb read_rb 1')
    assert response == 'Rollback index: 0'


@pytest.mark.buildconfigspec('cmd_avb')
def test_avb_is_unlocked(u_boot_console):
    """Test if device is in the unlocked state
    """

    response = u_boot_console.run_command('avb init mmc %s' % str(mmc_dev))
    assert response == ''

    response = u_boot_console.run_command('avb is_unlocked')
    assert response == 'Unlocked = 1'


@pytest.mark.buildconfigspec('cmd_avb')
@pytest.mark.buildconfigspec('cmd_mmc')
def test_avb_mmc_read(u_boot_console):
    """Test mmc read operation
    """
    response = u_boot_console.run_command('print kernel_addr_r')
    temp_addr = response.split("=")[-1].strip()
    temp_addr2 = hex(int(temp_addr, 16) + 0x2000)

    response = u_boot_console.run_command('mmc rescan; mmc dev %s 0' %
                                          str(mmc_dev))
    assert('is current device' in response)

    response = u_boot_console.run_command('mmc read %s 0x22 0x1' % temp_addr)
    assert('read: OK' in response)

    response = u_boot_console.run_command('avb init mmc %s' % str(mmc_dev))
    assert response == ''

    response = u_boot_console.run_command('avb read_part veeprom 0 100 %s' %
                                           temp_addr2)
    assert('Read 512 bytes' in response)

    # Now lets compare two buffers
    response = u_boot_console.run_command('cmp %s %s 40' %
                                          (temp_addr, temp_addr2))
    assert('64 word' in response)
