# - Config file for the oculus package
# It defines the following variables
#  oculus_INCLUDE_DIRS - include directories for oculus
#  oculus_LIBRARY_DIRS - directories for oculus library

# Compute paths
get_filename_component(oculus_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
set(oculus_INCLUDE_DIRS "F:/Projects/liboculus/include;D:/software/anaconda3/Library/include;/include")
set(oculus_LIBRARY_DIRS "F:/Projects/liboculus/build_clean/lib/$<CONFIG>")
set(oculus_LIBRARIES "oculus")
