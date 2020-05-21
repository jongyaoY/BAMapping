FIND_LIBRARY(DBoW2_LIBRARY DBoW2
    PATHS "/home/jojo/BAMapping/libs/dbow_lib/lib"
)
FIND_PATH(DBoW2_INCLUDE_DIR DBoW2Config.cmake
    PATHS "/home/jojo/BAMapping/libs/dbow_lib/include/DBoW2" 
)
SET(DBoW2_LIBRARIES ${DBoW2_LIBRARY})
SET(DBoW2_LIBS ${DBoW2_LIBRARY})
SET(DBoW2_INCLUDE_DIRS ${DBoW2_INCLUDE_DIR})
