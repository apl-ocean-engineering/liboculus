#  Copyright Olivier Parcollet 2010, Andrey Antipov 2013.
#
# This module looks for Blueprint Oculus SDK
#
# OCULUS_DIR can be provided as a hint.
#
# It sets up : OCULUS_SDK_ROOT
#
#

find_package(PkgConfig)

SET(TRIAL_PATHS
 $ENV{OCULUS_DIR}
 ${OCULUS_DIR}
)

find_path(OCULUS_SDK_ROOT Oculus/Oculus.h
         HINTS ${TRIAL_PATHS}
         DOC "Location OCULUS SDK"
        )

find_package_handle_standard_args(OCULUSSDK DEFAULT_MSG OCULUS_SDK_ROOT)
