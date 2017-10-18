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

#pragma once

#include <boost/asio.hpp>
#include <stdio.h>
#include "data_packet.h"

namespace simulator {
namespace communication {

/******************************* MessageHeadere *******************************/
class MessageHeader {
    using Socket = boost::asio::ip::tcp::socket;
public:
    MessageHeader() {}

    void read_from_socket(Socket& s);

    void write_to_socket(Socket& s);

    void make_header(size_t msg_size) {
        msg_size_ = msg_size;
    }

    size_t msg_size() { return msg_size_; }
        
private:
    size_t msg_size_;
    static size_t const header_size = sizeof(size_t);
};

/******************************** Communicators *******************************/
//// Communicator
class Communicator {
protected:
    using Buffer = simulator::util::BinaryBuffer;
    using BufferPtr = std::unique_ptr<Buffer>;
    using IOService = boost::asio::io_service;
    using Socket = boost::asio::ip::tcp::socket;

public:
    Communicator();

    virtual ~Communicator() {}

protected:
    virtual bool establish_connection() = 0;
    
    void close_connection();

    void send_msg();

    void receive_msg();

    template <typename T>
    void append_msg(const T& t) {
        msg_body_->append(t);
    }

    template <typename T, typename... Args>
    void append_msg(const T& t, Args... args) {
        msg_body_->append(t);
        append_msg(args...);
    }

    template <typename... Args>
    void send_msg_and_get_reply(const std::string& cmd,
                                const StatePacket* sim_data,
                                Args... args) {
        msg_body_->clear();
        append_msg(cmd, args...);
        if (sim_data) {
            sim_data->encode(*msg_body_);
        }
        send_msg();

        std::string reply;
        receive_msg();
        msg_body_->read(reply);
        CHECK_EQ(reply, cmd);
    }


    IOService io_service_;
    Socket socket_;
    MessageHeader msg_header_;
    BufferPtr msg_body_;
};

//// CommServer
class CommServer : public Communicator {
    using Acceptor = boost::asio::ip::tcp::acceptor; 
    using Endpoint = boost::asio::ip::tcp::endpoint;

public:
    CommServer(const int port_no);

    ~CommServer() {}

protected:
    virtual bool establish_connection() override;

private:
    int port_;
    Acceptor acceptor_;
};

//// CommClient
class CommClient : public Communicator {
    using Resolver = boost::asio::ip::tcp::resolver;
    using TCP_Query = boost::asio::ip::tcp::resolver::query;
    using TCP_Iterator = boost::asio::ip::tcp::resolver::iterator;

public:
    CommClient(const std::string& host, const int port_no); 
    
    ~CommClient() {}
 
protected:
    virtual bool establish_connection() override;

    std::string host_;
    int port_;
    Resolver resolver_;
    static const int MAX_ATTEMPTS;
};

}} // simulator::communication
