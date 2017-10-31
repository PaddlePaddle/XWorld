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

/******************************** MessageHeader *******************************/
class MessageHeader {
private:
    using Socket = boost::asio::ip::tcp::socket;
    using BinaryBuffer = simulator::util::BinaryBuffer;
    using BinaryBufferPtr = std::unique_ptr<BinaryBuffer>;
public:
    MessageHeader() {}

    void read_from_socket(Socket& s);

    void insert_into_msg(BinaryBufferPtr& msg_body) {
        msg_body->insert(0, msg_size_);
    }

    void make_header(size_t msg_size) {
        msg_size_ = msg_size;
    }

    size_t msg_size() { return msg_size_; }
        
private:
    void null_callback() {}
    size_t msg_size_;
    static size_t const header_size = sizeof(size_t);
};

/******************************** Communicators *******************************/
//// Communicator
class Communicator {
protected:
    using BinaryBuffer = simulator::util::BinaryBuffer;
    using BinaryBufferPtr = std::unique_ptr<BinaryBuffer>;
    using IOService = boost::asio::io_service;
    using Socket = boost::asio::ip::tcp::socket;

public:
    Communicator();

    virtual ~Communicator() {}

protected:
    virtual bool establish_connection() = 0;
    
    void close_connection();

    void deliver_msg();

    void receive_msg();

    template <typename... Args>
    void read_msg(Args... args) {}

    template <typename T, typename... Args>
    void read_msg(T& t, Args& ... rets) {
        msg_body_->read(t);
        read_msg(rets...);
    }

    template <typename... Args>
    void read_msg(StatePacket& sim_data, Args& ... rets) {
        read_msg(rets...);
        sim_data.decode(*msg_body_);
    }
    
    template <typename... Args>
    void compose_msg(const Args& ... args) {
        msg_body_->clear();
        append_msg(args...);
    }

    template <typename... Args>
    void compose_msg(const StatePacket& sim_data, const Args& ... args) {
        msg_body_->clear();
        append_msg(args...);
        sim_data.encode(*msg_body_);
    }

    IOService io_service_;
    Socket socket_;

private:
    template <typename... Args>
    void append_msg(const Args& ... args) {}

    template <typename T, typename... Args>
    void append_msg(const T& t, const Args& ... args) {
        msg_body_->append(t);
        append_msg(args...);
    }

    MessageHeader msg_header_;
    BinaryBufferPtr msg_body_;
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

    template <typename... Args>
    void call_remote_func(const std::string& func_name,
                          const StatePacket* sim_data,
                          Args... args) {
        if (sim_data) {
            compose_msg(*sim_data, func_name, args...);
        } else {
            compose_msg(func_name, args...);
        }
        deliver_msg();
        receive_msg();
        std::string reply;
        read_msg(reply);
        CHECK_EQ(reply, func_name);
    }

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
