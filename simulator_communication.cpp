// Copyright (c) 2017 Baidu Inc. All Rights Reserved.

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "simulator_communication.h"

namespace simulator {
namespace communication {

/******************************** Communicators *******************************/
//// Communicator
Communicator::Communicator() : socket_(io_service_) {
    msg_body_.reset(new BinaryBuffer());
}

void Communicator::close_connection() {
    socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_both);
    socket_.close();
}

void Communicator::deliver_msg() {
    size_t msg_size = msg_body_->size();
    msg_header_.make_header(msg_size);
    msg_header_.insert_into_msg(msg_body_);

    boost::asio::write(
            socket_, boost::asio::buffer(msg_body_->data(), msg_body_->size()));
}

void Communicator::receive_msg() {
    msg_header_.read_from_socket(socket_);
    size_t msg_size = msg_header_.msg_size();
    msg_body_->resize(msg_size);
    size_t bytes_read = boost::asio::read(
            socket_, boost::asio::buffer(msg_body_->data_mutable(), msg_size));
    CHECK_EQ(bytes_read, msg_size);
    msg_body_->rewind();
}

//// CommServer
CommServer::CommServer(const int port_no) :
        Communicator(),
        port_(port_no),
        acceptor_(io_service_, Endpoint(boost::asio::ip::tcp::v4(), port_)) {
}

bool CommServer::establish_connection() {
    acceptor_.accept(socket_);
    return true;
}

//// CommClient
const int CommClient::MAX_ATTEMPTS = 5;

CommClient::CommClient(const std::string& host, const int port_no) :
        Communicator(),
        host_(host), port_(port_no),
        resolver_(io_service_) {
}

bool CommClient::establish_connection() {
    TCP_Query query(boost::asio::ip::tcp::v4(),
                    host_.c_str(), std::to_string(port_).c_str());
    TCP_Iterator iterator = resolver_.resolve(query);

    bool connected = false;
    int attempts = 0;
    while (!connected && attempts < MAX_ATTEMPTS) {
        attempts++;
        try {
            boost::asio::connect(socket_, iterator);
            connected = true;
        } catch (boost::system::system_error const& e) {
            sleep(1);
        }
    }

    return connected;
}

}} // simulator::communication
