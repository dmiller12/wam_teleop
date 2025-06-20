
#include "udp_handler.h"
#include <boost/system/error_code.hpp>
#include <cstring>

template <size_t DOF>
UDPHandler<DOF>::UDPHandler(const std::string& remote_host, int send_port, int recv_port)
    : remote_host(remote_host)
    , send_port(send_port)
    , recv_port(recv_port)
    , stop_threads(false)
    , send_socket(io_context, boost::asio::ip::udp::v4())
    , recv_socket(io_context, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), recv_port)) {

    recv_thread = std::thread(&UDPHandler::receiveLoop, this);
    send_thread = std::thread(&UDPHandler::sendLoop, this);
}

template <size_t DOF>
UDPHandler<DOF>::~UDPHandler() {
    stop();
}

template <size_t DOF>
void UDPHandler<DOF>::stop() {
    stop_threads = true;
    io_context.stop();
    send_condition.notify_all();
    try {
        recv_socket.cancel();
        recv_socket.shutdown(boost::asio::ip::udp::socket::shutdown_both);
        recv_socket.close();
    } catch (...) {
        // Ignore any exceptions during socket cleanup
    }

    if (recv_thread.joinable())
        recv_thread.join();
    if (send_thread.joinable())
        send_thread.join();
}

template <size_t DOF>
boost::optional<typename UDPHandler<DOF>::ReceivedData> UDPHandler<DOF>::getLatestReceived() {
    std::lock_guard<std::mutex> lock(state_mutex);
    return latest_received;
}

template <size_t DOF>
void UDPHandler<DOF>::receiveLoop() {
    boost::asio::ip::udp::endpoint sender_endpoint;
    jp_type received_jp;
    char buffer[sizeof(double) * DOF];

    while (!stop_threads) {
        boost::system::error_code ec;
        size_t len = recv_socket.receive_from(boost::asio::buffer(buffer, sizeof(buffer)), sender_endpoint, 0, ec);

        if (ec == boost::asio::error::operation_aborted || len != sizeof(buffer))
            continue;

        std::memcpy(received_jp.data(), buffer, sizeof(double) * DOF);

        {
            std::lock_guard<std::mutex> lock(state_mutex);
            latest_received = ReceivedData{received_jp, std::chrono::steady_clock::now()};
        }
    }
    recv_socket.close();
}

template <size_t DOF>
void UDPHandler<DOF>::send(const jp_type& jp) {
    {
        std::lock_guard<std::mutex> lock(send_mutex);
        pending_send_jp = jp;
        new_data_available = true;
    }
    send_condition.notify_one();
}

template <size_t DOF>
void UDPHandler<DOF>::sendLoop() {
    boost::asio::ip::udp::endpoint remote_endpoint(boost::asio::ip::make_address(remote_host), send_port);

    while (!stop_threads) {
        std::unique_lock<std::mutex> lock(send_mutex);
        send_condition.wait(lock, [this] { return new_data_available || stop_threads; });

        if (stop_threads)
            break;

        new_data_available = false;
        jp_type data_to_send_jp = pending_send_jp;
        lock.unlock();

        char buffer[sizeof(double) * DOF];
        std::memcpy(buffer, data_to_send_jp.data(), sizeof(double) * DOF);

        boost::system::error_code ec;
        send_socket.send_to(boost::asio::buffer(buffer, sizeof(buffer)), remote_endpoint, 0, ec);
    }
    send_socket.close();
}

template class UDPHandler<7>; // For DOF=7
