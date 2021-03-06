# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(TextilesHand
                    CATEGORY device
                    TYPE roboticslab::TextilesHand
                    INCLUDE TextilesHand.hpp
                    DEFAULT ON
                    EXTRA_CONFIG WRAPPER=controlboardwrapper2)

if(NOT SKIP_TextilesHand)

    if(NOT YARP_VERSION VERSION_GREATER_EQUAL 3.4)
        set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this
    endif()

    yarp_add_plugin(TextilesHand TextilesHand.hpp
                                 DeviceDriverImpl.cpp
                                 IControlModeImpl.cpp
                                 IPositionDirectImpl.cpp)

    target_link_libraries(TextilesHand YARP::YARP_os
                                       YARP::YARP_dev
                                       ROBOTICSLAB::ColorDebug)

    target_compile_features(TextilesHand PUBLIC cxx_std_11)

    yarp_install(TARGETS TextilesHand
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
