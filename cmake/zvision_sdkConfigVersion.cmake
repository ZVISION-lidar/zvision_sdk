set (PACKAGE_VERSION "1.0.1")
message(=============================================================)
message("-- zvision_sdk Version : v${PACKAGE_VERSION}")
message(=============================================================)

# Check whether the requested PACKAGE_FIND_VERSION is compatible
if ("${PACKAGE_VERSION}" VERSION_LESS "${PACKAGE_FIND_VERSION}")
  set (PACKAGE_VERSION_COMPATIBLE FALSE)
else ()
  set (PACKAGE_VERSION_COMPATIBLE TRUE)
  if ("${PACKAGE_VERSION}" VERSION_EQUAL "${PACKAGE_FIND_VERSION}")
    set (PACKAGE_VERSION_EXACT TRUE)
  endif ()
endif ()
