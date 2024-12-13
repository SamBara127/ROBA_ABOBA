#include "rclcpp/rclcpp.hpp"
#include "service_interface_package/srv/summ_full_name.hpp" // Подключение интерфейса сервиса

using SummFullName = service_interface_package::srv::SummFullName;

class FullNameService : public rclcpp::Node
{
public:
    FullNameService() : Node("service_name")
    {
        // Создаём сервер
        service_ = this->create_service<SummFullName>(
            "SummFullName",
            std::bind(&FullNameService::handle_service, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Сервер SummFullName запущен");
    }

private:
    void handle_service(const std::shared_ptr<SummFullName::Request> request,
                        std::shared_ptr<SummFullName::Response> response)
    {
        // Склеиваем ФИО в одну строку
        response->full_name = request->last_name + " " + request->name + " " + request->first_name;
        RCLCPP_INFO(this->get_logger(), "Получено: %s %s %s -> Ответ: %s",
                    request->last_name.c_str(),
                    request->name.c_str(),
                    request->first_name.c_str(),
                    response->full_name.c_str());
    }

    rclcpp::Service<SummFullName>::SharedPtr service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FullNameService>());
    rclcpp::shutdown();
    return 0;
}
