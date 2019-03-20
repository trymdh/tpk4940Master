######################################################################################
## @package : CxCamLib
#  @file : cx_cam_nodemap_param.py
#  @brief Python example of using AT Camera library.
#  @copyright (c) 2017, Automation Technology GmbH.
#  @version 04.10.2017, AT: initial version
#
#  Demonstrates how to read and write parameters and how to get more detailed
#  information about a parameter.
######################################################################################

import numpy as np
from matplotlib import pyplot as plt
import cx.cx_cam as cam
import cx.cx_base as base

def print_info(hDev, name):
    val = "None"
    sym_entries = ""

    result, type = cam.cx_getParamInfo(hDev, cam.CX_PARAM_INFO_TYPE, name)
    if result != base.CX_STATUS_OK:
        print("cx_getParamInfo(Type) from %s returned error %d" % (name, result))
        return
    result, range = cam.cx_getParamInfo(hDev, cam.CX_PARAM_INFO_RANGE, name)
    if result != base.CX_STATUS_OK:
        print("cx_getParamInfo(Range) from %s returned error %d" % (name, result))
        return
    result, descr = cam.cx_getParamInfo(hDev, cam.CX_PARAM_INFO_DESCRIPTION, name)
    if result != base.CX_STATUS_OK:
        print("cx_getParamInfo(Description) from %s returned error %d" % (name, result))
        return
    result, tooltip = cam.cx_getParamInfo(hDev, cam.CX_PARAM_INFO_TOOLTIP, name)
    if result != base.CX_STATUS_OK:
        print("cx_getParamInfo(Tooltip) from %s returned error %d" % (name, result))
        return
    result, access = cam.cx_getParamInfo(hDev, cam.CX_PARAM_INFO_ACCESSS_MODE, name)
    if result != base.CX_STATUS_OK:
        print("cx_getParamInfo(AccessMode) from %s returned error %d" % (name, result))
        return
    result, visibility = cam.cx_getParamInfo(hDev, cam.CX_PARAM_INFO_VISIBILITY, name)
    if result != base.CX_STATUS_OK:
        print("cx_getParamInfo(Visibility) from %s returned error %d" % (name, result))
        return
    if type == cam.CX_PARAM_ENUM:
        result, sym_entries = cam.cx_getParamInfo(hDev, cam.CX_PARAM_INFO_ENUM_SYMBOLS, name)
        if result != base.CX_STATUS_OK:
            print("cx_getParamInfo(Flags) from %s returned error %d" % (name, result))
            return
    if access == cam.CX_PARAM_ACCESS_RO or access == cam.CX_PARAM_ACCESS_RW:
        result, val = cam.cx_getParam(hDev, name)
        if result != base.CX_STATUS_OK:
            print("cx_getParam(Value) from %s returned error %d" % (name, result))
            return
    if type == cam.CX_PARAM_CATEGORY:
        result, cat_children = cam.cx_getParamInfo(hDev, cam.CX_PARAM_INFO_CATEGORY_CHILDS, name)
        if result != base.CX_STATUS_OK:
            print("cx_getParam(Children) from %s returned error %d" % (name, result))
            return

    visibility_str = "Undefined"
    if visibility == cam.CX_PARAM_VISIBILITY_BEGINNER:
        visibility_str = "Beginner"
    if visibility == cam.CX_PARAM_VISIBILITY_EXPERT:
        visibility_str = "Expert"
    if visibility == cam.CX_PARAM_VISIBILITY_GURU:
        visibility_str = "Guru"
    if visibility == cam.CX_PARAM_VISIBILITY_INVISIBLE:
        visibility_str = "Invisible"

    access_str = ""
    if access == cam.CX_PARAM_ACCESS_RO:
        access_str="RO"
    elif access == cam.CX_PARAM_ACCESS_WO:
        access_str="WO"
    elif access == cam.CX_PARAM_ACCESS_RW:
        access_str="RW"
    elif access == cam.CX_PARAM_ACCESS_NOT_AVAILABLE:
        access_str="Not Available"
    elif access & cam.CX_PARAM_ACCESS_NOT_IMPLEMENTED:
        access_str="Not Implemented"

    type_str = "Unknown"
    if type == cam.CX_PARAM_INTEGER:
        type_str = "Integer"
    elif type == cam.CX_PARAM_FLOAT:
        type_str = "Float"
    elif type == cam.CX_PARAM_STRING:
        type_str = "String"
    elif type == cam.CX_PARAM_ENUM:
        type_str = "Enum"
    elif type == cam.CX_PARAM_BOOLEAN:
        type_str = "Boolean"
    elif type == cam.CX_PARAM_COMMAND:
        type_str = "Command"
    elif type == cam.CX_PARAM_CATEGORY:
        type_str = "Category"

    print("Information for %s" % (name))
    print("\tType: %s" % (type_str))
    print("\tDescription: %s"  % (descr))
    print("\tTooltip: %s" % (tooltip))
    print("\tAccess: %s" % (access_str))
    print("\tVisibility: %s" % (visibility_str))
    print("\tValue: %s" % (str(val)))

    if type == cam.CX_PARAM_ENUM:
        sym_list = tuple(sym_entries.split('\0')) # shows how to convert to a list...
        print("\tEntries: %s" % ', '.join(map(str, sym_list)))
        print("\tRange: %s" % ', '.join([str(a) for a in range]))
    elif type == cam.CX_PARAM_STRING:
        print("\tRange: %s" % (str(range)))
    elif type == cam.CX_PARAM_COMMAND:
        print("\tRange: %s" % (str(range)))
    elif type == cam.CX_PARAM_INTEGER:
        print("\tRange: %s..%s" % (str(range[0]), str(range[1])))
    elif type == cam.CX_PARAM_CATEGORY:
        child_list = tuple(cat_children.split('\0'))  # shows how to convert to a list...
        print("\tChildren: %s" % ', '.join(map(str, child_list)))


# find connected devices
result = cam.cx_dd_findDevices("", 2000, cam.CX_DD_USE_GEV | cam.CX_DD_USE_GEV_BROADCAST)
if result!=base.CX_STATUS_OK:
    print("cx_dd_findDevices returned error %d" % (result))
    exit(0)

# get number of found devices
result, numDD = cam.cx_dd_getNumFoundDevices()
if result!=base.CX_STATUS_OK:
    print("cx_dd_getNumFoundDevices returned error %d" % (result))
    exit(0)

# output number of found devices
print("Found %d devices." % (numDD))
if numDD==0:
    exit(0)

# get the uniform resource identifier (URI) for the first device
ddIndex = 0 # first device
result, uri = cam.cx_dd_getParam(ddIndex, "URI")    # get uri of found device at ddIndex
if result!=base.CX_STATUS_OK:
    print("cx_dd_getParam(%d, %s) returned error %d" % (ddIndex, "URI", result))
    exit(0)

# connect the camera with given URI, you can also open the camera with a valid URI without the discovering step above
# e.g. uri = "gev://169.254.239.221/?mac=00-50-C2-8E-DD-EE"
result, hDev = cam.cx_openDevice(uri)
if result!=base.CX_STATUS_OK:
    print("cx_openDevice(%s) returned error %d" % (uri, result))
    exit(0)

# read and print some nodemap perameters
#Integer
print_info(hDev, "GevHeartbeatTimeout")
#Float
print_info(hDev, "DeviceTemperature")
#String
print_info(hDev, "DeviceVendorName")
#Enum
print_info(hDev, "DeviceTemperatureSelector")
#Command
print_info(hDev, "AcquisitionStart")
#Category
print_info(hDev, "TransportLayerControl")
#Pixel format
print_info(hDev, "PixelFormat")
# write some nodemap paremeters
#Enum
sval = "Mainboard"
result = cam.cx_setParam(hDev, "DeviceTemperatureSelector", sval)   # this can fail if just one temperature selection is available

if result!=base.CX_STATUS_OK:
    print("cx_setParam(DeviceTemperatureSelector, %s) returned error %d" % (sval, result))

#close connection
result = cam.cx_closeDevice(hDev)
if result!=base.CX_STATUS_OK:
    print("cx_closeDevice returned error %d" % (result))
