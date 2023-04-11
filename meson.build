project('libstlinkv3_bridge_libusb', 'cpp',
    license: 'ST Ultimate Liberty License',
    license_files: 'LICENSE',
    meson_version: '>=1.1',
    version: '0.1',
    default_options: 'cpp_std=gnu++17'
    )

install_header_files = [
    'src/bridge/bridge.h',
    'src/bridge/stlink_fw_api_bridge.h',
    'src/bridge/stlink_fw_const_bridge.h',
    'src/common/STLinkUSBDriver.h',
    'src/common/stlink_device.h',
    'src/common/stlink_fw_api_common.h',
    'src/common/stlink_if_common.h',
    'src/common/stlink_type.h',
]

include_dirs = [
    'src/bridge',
    'src/common',
]

src_files = [
    'src/bridge/bridge.cpp',
    'src/common/stlink_device.cpp',
    'src/common/stlink_interface.cpp',
]

if get_option('buildtype').startswith('debug')
    add_project_arguments('-DUSING_ERRORLOG', language : 'cpp')
    install_header_files += 'src/error/ErrLog.h'
    include_dirs += 'src/error'
    src_files += 'src/error/ErrLog.cpp'
endif

# Our only dependency
# We need this to be found with the pkg-config lookup method specifically
# since we use this dependency object in the `requires` parameter when we
# generate a pkg-config file for this project.
libusb = dependency('libusb-1.0', method: 'pkg-config')

inc_dirs = include_directories(include_dirs)

# When installing, place all header files in an include directory with this
# name
install_subdir_name = 'stlinkv3-bridge-libusb'

# When installing, install these public-facing header files
install_headers(
    install_header_files,
    subdir: install_subdir_name
)

lib = library('stlinkv3-bridge-libusb',
    src_files,
    dependencies: libusb,
    include_directories: inc_dirs,
    implicit_include_directories: false,
    install: true
    )

# For ease of others using this library.
# When you install this library, this also generated a pkg-config file
# We need to explicitly require libusb publically since it is used
# in our public API
pkg = import('pkgconfig')
pkg.generate(lib, subdirs: install_subdir_name, requires: libusb)

# Now lets declare a dependency describing ourself so that this project
# may be used as a subproject in another Meson project
libstlinkv3_bridge_libusb_dep = declare_dependency(
    include_directories: inc_dirs,
    link_with: lib,
    # libusb is used in the public-api
    dependencies: libusb
    )

# An example application
subdir('example')
