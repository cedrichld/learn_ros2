#include "rclcpp/rclcpp.hpp"
#include "dxl_test/xl330_driver.hpp"

using namespace std::chrono_literals;

int main()
{
    int dxl_id_1 = 10;
    int dxl_id_2 = 20;

    auto driver = XL330Driver("/dev/ttyACM0");
    driver.init();

    std::this_thread::sleep_for(1s);

    // TEST VELOCITY CONTROL
    driver.activateWithVelocityMode(dxl_id_1);
    driver.activateWithVelocityMode(dxl_id_2);
    driver.setTargetVelocityRadianPerSec(dxl_id_1, 3.14159);
    driver.setTargetVelocityRadianPerSec(dxl_id_2, -3.14159);
    
    std::this_thread::sleep_for(5s);
    double velocity = driver.getVelocityRadianPerSec(dxl_id_1);
    std::cout << "Velocity motor 1: " << velocity << std::endl;
    driver.deactivate(dxl_id_1);
    driver.deactivate(dxl_id_2);

    // TEST POSITION CONTROL
    driver.activateWithPositionMode(dxl_id_1);
    driver.activateWithPositionMode(dxl_id_2);
    driver.setTargetPositionRadian(dxl_id_1, 1.57);
    driver.setTargetPositionRadian(dxl_id_2, -1.57);
    
    std::this_thread::sleep_for(3s);
    double position = driver.getPositionRadian(dxl_id_1);
    std::cout << "Position motor 1: " << position << std::endl;
    driver.deactivate(dxl_id_1);
    driver.deactivate(dxl_id_2);


    return 0;
}