#!/usr/bin/env python3

#
# Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
#
# SPDX-License-Identifier: BSD-3-Clause
#

# sudo pip3 install pyusb

import usb.core
import usb.util

# find our device
dev = usb.core.find(idVendor=0x0000, idProduct=0x0001)

# was it found?
if dev is None:
    raise ValueError('Device not found')

# get an endpoint instance
cfg = dev.get_active_configuration()
intf = cfg[(0, 0)]

outep = usb.util.find_descriptor(
    intf,
    # match the first OUT endpoint
    custom_match= \
        lambda e: \
            usb.util.endpoint_direction(e.bEndpointAddress) == \
            usb.util.ENDPOINT_OUT)

inep = usb.util.find_descriptor(
    intf,
    # match the first IN endpoint
    custom_match= \
        lambda e: \
            usb.util.endpoint_direction(e.bEndpointAddress) == \
            usb.util.ENDPOINT_IN)

assert inep is not None
assert outep is not None

'''test_string = "id1:"*720
outep.write(test_string)
test_string = "id2:"*720
outep.write(test_string)
test_string = "id3:"*720
outep.write(test_string)
test_string = "id4:"*720
outep.write(test_string)
test_string = "id5:"*720
outep.write(test_string)
test_string = "id6:"*720
outep.write(test_string)
test_string = "id7:"*720
outep.write(test_string)
test_string = "id8:"*720
outep.write(test_string)'''
test_string = "cmd:set_port_res,80,12"
outep.write(test_string)
from_device = inep.read(64)
print("Device Says: {}".format(''.join([chr(x) for x in from_device])))
test_string = "cmd:set_pixel_interval,6"
outep.write(test_string)
from_device = inep.read(64)
print("Device Says: {}".format(''.join([chr(x) for x in from_device])))
