#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <iostream>
#include <optional>  // 新增头文件

#include "Eigen/Dense"
#include "inverted_pendulum.h"
#include "tools.h"
#include "pid.h"
#include "lqr.h"

int main() {
  // 初始化窗口（SFML 3.0 需要 Vector2u 参数）
  sf::RenderWindow window(sf::VideoMode(sf::Vector2u(640, 480)), "Inverted Pendulum");

  // 初始条件（保持不变）
  const double p_0 = 0;
  const double theta_0 = -5;
  Eigen::VectorXd x_0(4);
  x_0 << p_0, to_radians(theta_0), 0, 0;

  // PID 参数（保持不变）
  const double kp = 100.0F;
  const double ki = 50.0F;
  const double kd = 10.0F;

  // 创建模型和控制器（保持不变）
  InvertedPendulum *ptr = new InvertedPendulum(x_0);
  PID *c_ptr = new PID();
  c_ptr->Init(kp, ki, kd);

  // LQR 控制器（保持不变）
  LQR optimal;
  ptr->Linearize();
  optimal.A_ = ptr->A_;
  optimal.B_ = ptr->B_;
  optimal.Q_ = Eigen::MatrixXd::Identity(4, 4);
  optimal.Q_(0, 0) = 10;
  optimal.R_ = Eigen::MatrixXd::Identity(1, 1);
  optimal.Compute();

  //在这里改使用pid还是lqr
  bool pid = true;  //选择pid
  //bool pid = false;//选择lqr

  // 加载字体（SFML 3.0 使用 openFromFile）
  sf::Font font;
  if (!font.openFromFile("Roboto-Regular.ttf")) {  // 修改方法名
    std::cout << "Failed to load font!\n";
  }

  // Create text to display simulation time
  sf::Text text(font);
  text.setCharacterSize(24);
  const sf::Color grey = sf::Color(0x7E, 0x7E, 0x7E);
  text.setFillColor(grey);
  text.setPosition(sf::Vector2f(480.0F, 360.0F));

  // Create text to display controller type
  sf::Text type(font);
  type.setCharacterSize(24);
  const sf::Color turquoise = sf::Color(0x06, 0xC2, 0xAC);
  type.setFillColor(turquoise);
  type.setPosition(sf::Vector2f(480.0F, 384.0F));

  // 创建轨道（坐标设置使用 Vector2f）
  sf::RectangleShape track(sf::Vector2f(640.0F, 2.0F));
  track.setOrigin(sf::Vector2f(320.0F, 1.0F));  // 参数改为 Vector2f
  track.setPosition(sf::Vector2f(320.0F, 240.0F));
  const sf::Color light_grey = sf::Color(0xAA, 0xAA, 0xAA);
  track.setFillColor(light_grey);

  // 创建小车（坐标设置使用 Vector2f）
  sf::RectangleShape cart(sf::Vector2f(100.0F, 100.0F));
  cart.setOrigin(sf::Vector2f(50.0F, 50.0F));  // 参数改为 Vector2f
  cart.setPosition(sf::Vector2f(320.0F, 240.0F));
  cart.setFillColor(sf::Color::Black);

  // 创建摆杆（旋转使用 sf::degrees）
  sf::RectangleShape pole(sf::Vector2f(20.0F, 200.0F));
  pole.setOrigin(sf::Vector2f(10.0F, 200.0F));  // 参数改为 Vector2f
  pole.setPosition(sf::Vector2f(320.0F, 240.0F));
  pole.setRotation(sf::degrees(-theta_0));  // 使用 sf::degrees
  const sf::Color brown = sf::Color(0xCC, 0x99, 0x66);
  pole.setFillColor(brown);

  sf::Clock clock;

  while (window.isOpen()) {
    // 事件处理（SFML 3.0 新 API）
    while (const std::optional event = window.pollEvent())
    {
        if (event->is<sf::Event::Closed>())
        {
            window.close();
        }
    }
    // 更新仿真状态（保持不变）
    sf::Time elapsed = clock.getElapsedTime();
    const float time = elapsed.asSeconds();
    const std::string msg = std::to_string(time);
    text.setString("Time   " + msg.substr(0, msg.find('.') + 2));
    const std::string action = pid ? "Action PID" : "Action LQR";
    type.setString(action);
    
    if (time < 15) {
      double u = 0;
      if (pid) {
        double angle = ptr->GetState()(1);
        double error = 0.0F - angle;
        c_ptr->UpdateError(time, error);
        u = c_ptr->TotalError();
      } else {
        u = optimal.Control(ptr->GetState())(0, 0);
      }
      ptr->Update(time, u);
    } else {
      delete ptr;
      delete c_ptr;
      ptr = new InvertedPendulum(x_0);
      c_ptr = new PID();
      c_ptr->Init(kp, ki, kd);
      clock.restart();
      pid = !pid;  // 简化逻辑
    }

    Eigen::VectorXd x = ptr->GetState();

    // 更新图形位置（使用 Vector2f 和 sf::degrees）
    cart.setPosition(sf::Vector2f(
      320.0F + 100 * static_cast<float>(x(0)),  // 显式类型转换
      240.0F
    ));
    pole.setPosition(sf::Vector2f(
      320.0F + 100 * static_cast<float>(x(0)),
      240.0F
    ));
    pole.setRotation(sf::degrees(-to_degrees(x(1))));  // 双重转换确保类型安全

    // 渲染（保持不变）
    window.clear(sf::Color::White);
    window.draw(track);
    window.draw(cart);
    window.draw(pole);
    window.draw(text);
    window.draw(type);
    window.display();
  }
  
  // 清理资源
  delete ptr;
  delete c_ptr;
  return 0;
}