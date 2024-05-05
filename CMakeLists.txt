idf_component_register(SRCS CanBus.cpp LingKongMotor.cpp
                       REQUIRES esp_system esp_timer driver log 
                       INCLUDE_DIRS "."
                       )