cmake_minimum_required(VERSION 3.12)

project(inverted_pendulum)

# 设置C++标准
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加SFML安装路径到CMAKE_PREFIX_PATH
list(APPEND CMAKE_PREFIX_PATH "D:/Program Files/SFML-3.0.0")

# 添加库目录
add_subdirectory(src)

# 配置绘图可执行文件（sciplot）
find_package(sciplot)
if(sciplot_FOUND)
    target_compile_features(sciplot::sciplot INTERFACE cxx_std_17)
    add_executable(plot src/main.cpp)
    target_link_libraries(plot sciplot::sciplot source)
else()
    message(WARNING "sciplot not found, plot executable will not be built")
endif()

# 配置测试可执行文件（GTest）
find_package(GTest)
if(GTest_FOUND)
    enable_testing()
    add_executable(test tests/main.cpp tests/inverted_pendulum_test.cpp)
    target_link_libraries(test source GTest::GTest GTest::Main)
else()
    message(WARNING "GTest not found, test executable will not be built")
endif()

# 复制资源文件
configure_file(res/Roboto-Regular.ttf Roboto-Regular.ttf COPYONLY)

# 配置游戏可执行文件（SFML）
find_package(SFML COMPONENTS Graphics Window System REQUIRED)
add_executable(cartpole game/main.cpp)
target_link_libraries(cartpole source 
    SFML::Graphics 
    SFML::Window 
    SFML::System
)

# 示例可执行文件
add_executable(example examples/lqr_design.cpp)
target_link_libraries(example source)
