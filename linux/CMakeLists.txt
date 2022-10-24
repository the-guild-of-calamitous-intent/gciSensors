# LIST(APPEND files
#     rpi.cpp
# )

add_library(wirelib
    Wire.cpp
    Arduino.cpp
)

target_link_libraries(wirelib
    PRIVATE
        i2c
)

