#pragma once

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

namespace ros_gateserver
{
namespace net = boost::asio;
namespace beast = boost::beast;
namespace websocket = beast::websocket;
using tcp = net::ip::tcp;

// All socket operations run on one io_context thread. ROS supplies a locked snapshot.
class WebSocketServer
{
    class Session : public std::enable_shared_from_this<Session>
    {
    public:
        Session(tcp::socket socket,
            std::function<std::string()> snapshot,
            std::chrono::milliseconds interval,
            bool binary)
            : socket_(std::move(socket)),
              timer_(socket_.get_executor()),
              snapshot_(std::move(snapshot)),
              interval_(interval),
              binary_(binary)
        {
        }

        void start()
        {
            // 设置连接超时。
            socket_.set_option(
                websocket::stream_base::timeout::suggested(beast::role_type::server));
            socket_.async_accept(
                [self = shared_from_this()](beast::error_code error)
                {
                    if (error)
                    {
                        return;
                    }

                    // 接收链处理客户端消息与断开；发送链按周期读取最新状态快照。
                    self->read();
                    self->write();
                });
        }

    private:
        void stop()
        {
            stopped_ = true;
            timer_.cancel();

            beast::error_code ignored;
            socket_.next_layer().cancel(ignored);
            socket_.next_layer().close(ignored);
        }

        void read()
        {
            socket_.async_read(input_,
                [self = shared_from_this()](beast::error_code error, size_t)
                {
                    if (error)
                    {
                        self->stop();
                        return;
                    }

                    self->input_.consume(self->input_.size());
                    self->read();
                });
        }

        void write()
        {
            if (stopped_)
            {
                return;
            }

            output_ = snapshot_();
            // 字符串可保存任意字节；protobuf 使用 binary 帧，长度由 buffer 保留。
            socket_.binary(binary_);
            socket_.async_write(net::buffer(output_),
                [self = shared_from_this()](beast::error_code error, size_t)
                {
                    if (error)
                    {
                        self->stop();
                        return;
                    }

                    self->timer_.expires_after(self->interval_);
                    self->timer_.async_wait(
                        [self](beast::error_code timer_error)
                        {
                            if (!timer_error)
                            {
                                self->write();
                            }
                        });
                });
        }

        websocket::stream<tcp::socket> socket_;
        net::steady_timer timer_;
        beast::flat_buffer input_;
        std::function<std::string()> snapshot_;
        std::chrono::milliseconds interval_;
        std::string output_;
        bool stopped_{false};
        bool binary_{false};
    };

public:
    WebSocketServer(const std::string& host,
        unsigned short port,
        std::function<std::string()> snapshot,
        std::chrono::milliseconds interval,
        bool binary = false)
        : acceptor_(context_, tcp::endpoint(net::ip::make_address(host), port)),
          snapshot_(std::move(snapshot)),
          interval_(interval),
          binary_(binary)
    {
        accept();
        thread_ = std::thread(
            [this]
            {
                context_.run();
            });
    }

    ~WebSocketServer()
    {
        context_.stop();
        if (thread_.joinable())
        {
            thread_.join();
        }
    }

private:
    void accept()
    {
        acceptor_.async_accept(
            [this](beast::error_code error, tcp::socket socket)
            {
                if (!error)
                {
                    std::make_shared<Session>(std::move(socket), snapshot_, interval_, binary_)
                        ->start();
                }

                if (acceptor_.is_open())
                {
                    accept();
                }
            });
    }

    net::io_context context_;
    tcp::acceptor acceptor_;
    std::function<std::string()> snapshot_;
    std::chrono::milliseconds interval_;
    bool binary_{false};
    std::thread thread_;
};
} // namespace ros_gateserver
