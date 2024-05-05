idf_component_register(SRCS main.cpp
                       REQUIRES esp_system log driver esp_timer
                       REQUIRES canbus
                       INCLUDE_DIRS ".")
