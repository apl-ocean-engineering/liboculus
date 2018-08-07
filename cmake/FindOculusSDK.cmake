#  Copyright Olivier Parcollet 2010, Andrey Antipov 2013.
#
# This module looks for Blackmagic Decklink SDK
#
# BLACKMAGIC_ROOT can be provided as a hint.
#
# It sets up : BLACKMAGIC_SRCS
#              BLACKMAGIC_INCLUDE_DIR
#

find_package(PkgConfig)

# They use per-OS trees
if(LINUX OR BLACKMAGIC_FORCE_LINUX)
  SET( BM_ARCH "Linux" )
  SET( BLACKMAGIC_LIBS dl )
elseif(APPLE)
  SET( BM_ARCH "Mac" )
  SET( BLACKMAGIC_LIBS "-framework CoreFoundation" )
else()
  SET( BM_ARCH "Win" )
endif()

SET(TRIAL_PATHS
 $ENV{BLACKMAGIC_ROOT}/${BM_ARCH}/include
 ${BLACKMAGIC_ROOT}/${BM_ARCH}/include
 $ENV{BLACKMAGIC_ROOT}/include
 ${BLACKMAGIC_ROOT}/include
)

find_path(BLACKMAGIC_INCLUDE_DIR DeckLinkAPI.h
          HINTS ${TRIAL_PATHS}
          DOC "Include for Blackmagic SDK"
         )
find_path(BLACKMAGIC_SRCS DeckLinkAPIDispatch.cpp
         HINTS ${TRIAL_PATHS}
         DOC "Location of DecklinkAPIDispatch source file for Blackmagic SDK"
        )

set(BLACKMAGIC_INCLUDE_DIR ${BLACKMAGIC_INCLUDE_DIR} )
set(BLACKMAGIC_SRCS ${BLACKMAGIC_SRCS}/DeckLinkAPIDispatch.cpp )

find_package_handle_standard_args(BlackmagicSDK DEFAULT_MSG BLACKMAGIC_INCLUDE_DIR)
