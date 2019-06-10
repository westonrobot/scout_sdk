/* 
 * async_can.cpp
 * 
 * Created on: Jun 10, 2019 02:25
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 * Copyright (c) 2016 UAVCAN Team
 */

// This is needed to enable necessary declarations in sys/
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "async_io/async_can.hpp"

#include <net/if.h>
#include <poll.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <cassert>
#include <iostream>

#include "asyncio_utils.hpp"

using namespace wescore;

using asio::buffer;
using asio::io_service;
using std::error_code;

std::atomic<size_t> ASyncCAN::conn_id_counter{0};

ASyncCAN::ASyncCAN(std::string device) : tx_total_bytes(0),
                                         rx_total_bytes(0),
                                         last_tx_total_bytes(0),
                                         last_rx_total_bytes(0),
                                         last_iostat(steady_clock::now()),
                                         tx_in_progress(false),
                                         tx_q{},
                                         rx_buf{},
                                         io_service(),
                                         stream(io_service)
{
    conn_id = conn_id_counter.fetch_add(1);
    open(device);
}

ASyncCAN::~ASyncCAN()
{
    close();
}

void ASyncCAN::open(std::string device)
{
    const size_t iface_name_size = strlen(device.c_str()) + 1;
    if (iface_name_size > IFNAMSIZ)
        return;

    can_fd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (can_fd_ < 0)
        return;

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    memcpy(ifr.ifr_name, device.c_str(), iface_name_size);

    const int ioctl_result = ioctl(can_fd_, SIOCGIFINDEX, &ifr);
    if (ioctl_result < 0)
        close();

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    const int bind_result = bind(can_fd_, (struct sockaddr *)&addr, sizeof(addr));
    if (bind_result < 0)
        close();
    ;

    can_interface_opened_ = true;

    // NOTE: shared_from_this() should not be used in constructors

    // give some work to io_service before start
    stream.assign(can_fd_);
    io_service.post(std::bind(&ASyncCAN::do_read, this, std::ref(rcv_frame), std::ref(stream)));

    // run io_service for async io
    io_thread = std::thread([this]() {
        set_this_thread_name("mcan%zu", conn_id);
        io_service.run();
    });
}

void ASyncCAN::close()
{
    // lock_guard lock(mutex);
    // if (!is_open())
    //     return;

    const int close_result = ::close(can_fd_);
    can_fd_ = -1;

    io_service.stop();

    if (io_thread.joinable())
        io_thread.join();

    io_service.reset();

    can_interface_opened_ = false;

    if (port_closed_cb)
        port_closed_cb();
}

ASyncCAN::IOStat ASyncCAN::get_iostat()
{
    std::lock_guard<std::recursive_mutex> lock(iostat_mutex);
    IOStat stat;

    stat.tx_total_bytes = tx_total_bytes;
    stat.rx_total_bytes = rx_total_bytes;

    auto d_tx = stat.tx_total_bytes - last_tx_total_bytes;
    auto d_rx = stat.rx_total_bytes - last_rx_total_bytes;
    last_tx_total_bytes = stat.tx_total_bytes;
    last_rx_total_bytes = stat.rx_total_bytes;

    auto now = steady_clock::now();
    auto dt = now - last_iostat;
    last_iostat = now;

    float dt_s = std::chrono::duration_cast<std::chrono::seconds>(dt).count();

    stat.tx_speed = d_tx / dt_s;
    stat.rx_speed = d_rx / dt_s;

    return stat;
}

void ASyncCAN::iostat_tx_add(size_t bytes)
{
    tx_total_bytes += bytes;
}

void ASyncCAN::iostat_rx_add(size_t bytes)
{
    rx_total_bytes += bytes;
}

void ASyncCAN::send_frame(const can_frame &tx_frame)
{
    // if (!is_open())
    // {
    //     std::cerr << "send: channel closed!"
    //               << " connection id: " << conn_id << std::endl;
    //     return;
    // }

    // {
    //     lock_guard lock(mutex);

    //     if (tx_q.size() >= MAX_TXQ_SIZE)
    //         throw std::length_error("ASyncCAN::send_bytes: TX queue overflow");

    //     tx_q.emplace_back(bytes, length);
    // }
    // io_service.post(std::bind(&ASyncCAN::do_write, shared_from_this(), true));

    // TODO implement a tx buffer
    stream.async_write_some(asio::buffer(&tx_frame, sizeof(tx_frame)),
                            [](error_code error, size_t bytes_transferred) {
                                std::cout << "frame sent" << std::endl;
                            });
}

void ASyncCAN::call_receive_callback(uint8_t *buf, const std::size_t bufsize, std::size_t bytes_received)
{
    assert(bufsize >= bytes_received);

    // keep track of statistics
    iostat_rx_add(bytes_received);

    // call the actual parser
    if (receive_cb)
        receive_cb(buf, bufsize, bytes_received);
    else
        default_receive_callback(buf, bufsize, bytes_received);
}

void ASyncCAN::default_receive_callback(uint8_t *buf, const size_t bufsize, size_t bytes_received)
{
    // do nothing
    std::cerr << "no callback function set" << std::endl;
}

void ASyncCAN::do_read(struct can_frame &rec_frame, asio::posix::basic_stream_descriptor<> &stream)
{
    auto sthis = shared_from_this();
    stream.async_read_some(
        asio::buffer(&rcv_frame, sizeof(rcv_frame)),
        [sthis](error_code error, size_t bytes_transferred) {
            if (error)
            {
                std::cerr << "read error in connection " << sthis->conn_id << " : "
                          << error.message().c_str() << std::endl;
                sthis->close();
                return;
            }
            std::cout << std::hex << sthis->rcv_frame.can_id << "  ";
            for (int i = 0; i < sthis->rcv_frame.can_dlc; i++)
                std::cout << std::hex << int(sthis->rcv_frame.data[i]) << " ";
            std::cout << std::dec << std::endl;
            // sthis->call_receive_callback(sthis->rx_buf.data(), sthis->rx_buf.size(), bytes_transferred);
            sthis->do_read(std::ref(sthis->rcv_frame), std::ref(sthis->stream));
        });
}

void ASyncCAN::do_write(bool check_tx_state)
{
    if (check_tx_state && tx_in_progress)
        return;

    lock_guard lock(mutex);
    if (tx_q.empty())
        return;

    tx_in_progress = true;
    auto sthis = shared_from_this();
    auto &buf_ref = tx_q.front();
    stream.async_write_some(
        buffer(buf_ref.dpos(), buf_ref.nbytes()),
        [sthis, &buf_ref](error_code error, size_t bytes_transferred) {
            assert(bytes_transferred <= buf_ref.len);

            if (error)
            {
                std::cerr << "write error in connection " << sthis->conn_id << " : "
                          << error.message().c_str() << std::endl;
                sthis->close();
                return;
            }

            sthis->iostat_tx_add(bytes_transferred);
            lock_guard lock(sthis->mutex);

            if (sthis->tx_q.empty())
            {
                sthis->tx_in_progress = false;
                return;
            }

            buf_ref.pos += bytes_transferred;
            if (buf_ref.nbytes() == 0)
            {
                sthis->tx_q.pop_front();
            }

            if (!sthis->tx_q.empty())
                sthis->do_write(false);
            else
                sthis->tx_in_progress = false;
        });
}

// //---------------------------------------------------------------------------------------//

// int socketcanInit(SocketCANInstance *out_ins, const char *can_iface_name)
// {
//     const size_t iface_name_size = strlen(can_iface_name) + 1;
//     if (iface_name_size > IFNAMSIZ)
//     {
//         goto fail0;
//     }

//     const int fd = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
//     if (fd < 0)
//     {
//         goto fail0;
//     }

//     struct ifreq ifr;
//     memset(&ifr, 0, sizeof(ifr));
//     memcpy(ifr.ifr_name, can_iface_name, iface_name_size);

//     const int ioctl_result = ioctl(fd, SIOCGIFINDEX, &ifr);
//     if (ioctl_result < 0)
//     {
//         goto fail1;
//     }

//     struct sockaddr_can addr;
//     memset(&addr, 0, sizeof(addr));
//     addr.can_family = AF_CAN;
//     addr.can_ifindex = ifr.ifr_ifindex;

//     const int bind_result = bind(fd, (struct sockaddr *)&addr, sizeof(addr));
//     if (bind_result < 0)
//     {
//         goto fail1;
//     }

//     out_ins->fd = fd;
//     return 0;

// fail1:
//     close(fd);
// fail0:
//     return -1;
// }

// int socketcanClose(SocketCANInstance *ins)
// {
//     const int close_result = close(ins->fd);
//     ins->fd = -1;
//     return close_result;
// }

// int socketcanTransmit(SocketCANInstance *ins, const CanardCANFrame *frame, int timeout_msec)
// {
//     struct pollfd fds;
//     memset(&fds, 0, sizeof(fds));
//     fds.fd = ins->fd;
//     fds.events |= POLLOUT;

//     const int poll_result = poll(&fds, 1, timeout_msec);
//     if (poll_result < 0)
//     {
//         return -1;
//     }
//     if (poll_result == 0)
//     {
//         return 0;
//     }
//     if ((fds.revents & POLLOUT) == 0)
//     {
//         return -1;
//     }

//     struct can_frame transmit_frame;
//     memset(&transmit_frame, 0, sizeof(transmit_frame));
//     transmit_frame.can_id = frame->id; // TODO: Map flags properly
//     transmit_frame.can_dlc = frame->data_len;
//     memcpy(transmit_frame.data, frame->data, frame->data_len);

//     const ssize_t nbytes = write(ins->fd, &transmit_frame, sizeof(transmit_frame));
//     if (nbytes < 0 || (size_t)nbytes != sizeof(transmit_frame))
//     {
//         return -1;
//     }

//     return 1;
// }

// int socketcanReceive(SocketCANInstance *ins, CanardCANFrame *out_frame, int timeout_msec)
// {
//     struct pollfd fds;
//     memset(&fds, 0, sizeof(fds));
//     fds.fd = ins->fd;
//     fds.events |= POLLIN;

//     const int poll_result = poll(&fds, 1, timeout_msec);
//     if (poll_result < 0)
//     {
//         return -1;
//     }
//     if (poll_result == 0)
//     {
//         return 0;
//     }
//     if ((fds.revents & POLLIN) == 0)
//     {
//         return -1;
//     }

//     struct can_frame receive_frame;
//     const ssize_t nbytes = read(ins->fd, &receive_frame, sizeof(receive_frame));
//     if (nbytes < 0 || (size_t)nbytes != sizeof(receive_frame))
//     {
//         return -1;
//     }

//     if (receive_frame.can_dlc > CAN_MAX_DLEN) // Appeasing Coverity Scan
//     {
//         return -1;
//     }

//     out_frame->id = receive_frame.can_id; // TODO: Map flags properly
//     out_frame->data_len = receive_frame.can_dlc;
//     memcpy(out_frame->data, &receive_frame.data, receive_frame.can_dlc);

//     return 1;
// }

// int socketcanGetSocketFileDescriptor(const SocketCANInstance *ins)
// {
//     return ins->fd;
// }
