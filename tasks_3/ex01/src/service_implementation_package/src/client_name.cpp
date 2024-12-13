#include "rclcpp/rclcpp.hpp"
#include "service_interface_package/srv/summ_full_name.hpp" // Подключение интерфейса сервиса

using SummFullName = service_interface_package::srv::SummFullName;

class FullNameClient : public rclcpp::Node
{
public:
    FullNameClient() : Node("client_name")
    {
        // Создаём клиент
        client_ = this->create_client<SummFullName>("SummFullName");
        RCLCPP_INFO(this->get_logger(), "Клиент готов к вызову сервиса");
    }

    void send_request(const std::string &last_name, const std::string &name, const std::string &first_name)
    {
        // Проверяем доступность сервиса
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Ожидание сервиса...");
        }

        // Создаём запрос
        auto request = std::make_shared<SummFullName::Request>();
        request->last_name = last_name;
        request->name = name;
        request->first_name = first_name;

        // Ожидание ответа
        // Отправляем запрос и ожидаем ответа
        auto future = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Ответ сервиса: %s", response->full_name.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Ошибка вызова сервиса SummFullName");
        }
            // if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
        // {
        //     try
        //     {
        //         auto response = future.get();
        //         RCLCPP_INFO(this->get_logger(), "Ответ сервиса: %s", response->full_name.c_str());
        //     }
        //     catch (const std::exception &e)
        //     {
        //         RCLCPP_ERROR(this->get_logger(), "Ошибка получения ответа от сервиса: %s", e.what());
        //     }
        // }
        // else
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Ответ от сервиса не был получен в течение 5 секунд");
        // }
    }

private:
    rclcpp::Client<SummFullName>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc != 4)
    {
        std::cerr << "Использование: ros2 run service_implementation_package client_name <Фамилия> <Имя> <Отчество>" << std::endl;
        return 1;
    }

    auto node = std::make_shared<FullNameClient>();
    node->send_request(argv[1], argv[2], argv[3]);

    rclcpp::shutdown();
    return 0;
}
