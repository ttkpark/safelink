idf_component_register(SRCS "alert_manager.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver freertos audio_manager safety_analyzer) 