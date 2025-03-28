#pragma once
#include <boost/asio.hpp>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <Eigen/Dense>
#include <boost/optional.hpp>
#include <chrono>

template <size_t DOF>
class UDPHandler {
public:
    using jp_type = Eigen::Matrix<double, DOF, 1>;
    using jv_type = Eigen::Matrix<double, DOF, 1>;

    struct ReceivedData {
        jp_type jp;
        jp_type jv;
        std::chrono::steady_clock::time_point timestamp;
    };

    UDPHandler(const std::string& remote_host, int send_port, int recv_port);
    ~UDPHandler();

    void stop();
    boost::optional<ReceivedData> getLatestReceived();
    void send(const jp_type& jp, const jv_type& jv);

private:
    std::string remote_host;
    int send_port, recv_port;
    std::atomic<bool> stop_threads;

    boost::asio::io_context io_context;
    boost::asio::ip::udp::socket send_socket;
    boost::asio::ip::udp::socket recv_socket;
    std::thread recv_thread, send_thread;

    std::mutex state_mutex, send_mutex;
    std::condition_variable send_condition;

    jp_type pending_send_jp;
    jv_type pending_send_jv;
    boost::optional<ReceivedData> latest_received;
    bool new_data_available = false;

    void receiveLoop();
    void sendLoop();
};
