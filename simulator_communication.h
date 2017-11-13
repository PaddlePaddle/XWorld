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
/**
 * Manage the message header that will be added in front of the message to be
 * sent.
 *
 * Message header is a fixed-size byte array that encodes necessary information
 * for later process of the actual message. Currently, message header is simply
 * the size of the message. To put more content in message header, one only
 * needs to change the member function `make_header`.
 */
class MessageHeader {
private:
    using Socket = boost::asio::ip::tcp::socket;
    using BinaryBuffer = simulator::util::BinaryBuffer;
    using BinaryBufferPtr = std::unique_ptr<BinaryBuffer>;
public:
    MessageHeader() {}
    /**
     * Read message header from socket.
     *
     * This function should be called before reading the message content.
     */
    void read_from_socket(Socket& s) {
        try {
            size_t bytes_read = boost::asio::read(
                    s, boost::asio::buffer((char*)&msg_size_, header_size));
            CHECK_EQ(bytes_read, header_size);
        } catch (boost::system::system_error const & e) {
            LOG(FATAL) << "asio error occured when reading message header";
        }
    }
    /**
     * Add message header content in from of the message.
     */
    void insert_into_msg(BinaryBufferPtr& msg_body) {
        msg_body->insert(0, msg_size_);
    }
    /**
     * Construct message header.
     */
    void make_header(size_t msg_size) {
        msg_size_ = msg_size;
    }
    /**
     * Return the size (in terms of bytes) of message (not including message
     * head).
     */
    size_t msg_size() { return msg_size_; }

private:
    size_t msg_size_;
    static size_t const header_size = sizeof(size_t);
};

/******************************** Communicators *******************************/
/**
 * Communicator provides the interface for composing and interpreting messages
 * and sending and receiving messages through socket.
 *
 * Communicator is designed specifically for our game simulation. It assumes
 * that users from both endpoints (e.g., server and client) have consensus on
 * the message format.
 */
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
    /**
     * Establish TCP/IP connection with a remote endpoint.
     *
     * Users should implement this method for both server and client sides. The
     * function returns true if connection is established, false otherwise.
     */
    virtual bool establish_connection() = 0;
    /**
     * Shutdown and close socket.
     */
    void close_connection();
    /**
     * Send message to through an established socket.
     *
     * The call will block until the message is transferred.
     */
    void deliver_msg();
    /**
     * Receive message from an established socket.
     *
     * This function does not interpret message. Users should use `read_msg` to
     * get content from the message.
     */
    void receive_msg();
    /**
     * Get content from message.
     */
    template <typename... Args>
    void read_msg(Args&... args) {}
    /**
     * Get content from message.
     */
    template <typename T, typename... Args>
    void read_msg(T& t, Args& ... rets) {
        msg_body_->read(t);
        read_msg(rets...);
    }
    /**
     * Get content from message.
     *
     * @param[out]   sim_data    simulation data
     * @param[out]   rets        rest of data to get from message
     */
    template <typename... Args>
    void read_msg(StatePacket& sim_data, Args& ... rets) {
        read_msg(rets...);
        sim_data.decode(*msg_body_);
    }
    /**
     * Compose message with provided inputs.
     */
    template <typename... Args>
    void compose_msg(const Args& ... args) {
        msg_body_->clear();
        append_msg(args...);
    }
    /**
     * Compose message with provided inputs.
     *
     * @param[in]   sim_data    simulation data
     * @param[in]   rets        rest of data to put into message
     */
    template <typename... Args>
    void compose_msg(const StatePacket& sim_data, const Args& ... args) {
        msg_body_->clear();
        append_msg(args...);
        sim_data.encode(*msg_body_);
    }

    IOService io_service_;
    Socket socket_;

private:
    /**
     * Utility function called by `compose_msg` to add data into the back
     * of message.
     */
    template <typename... Args>
    void append_msg(const Args& ... args) {}
    /**
     * Utility function called by `compose_msg` to add data into the back
     * of message.
     */
    template <typename T, typename... Args>
    void append_msg(const T& t, const Args& ... args) {
        msg_body_->append(t);
        append_msg(args...);
    }

    MessageHeader msg_header_;
    BinaryBufferPtr msg_body_;
};

/**
 * Communicator on the server (main process) side.
 */
class CommServer : public Communicator {
    using Acceptor = boost::asio::ip::tcp::acceptor;
    using Endpoint = boost::asio::ip::tcp::endpoint;

public:
    CommServer(const int port_no);

    ~CommServer() {}

protected:
    /**
     * Set up listening port and wait for connection from client side.
     */
    virtual bool establish_connection() override;
    /**
     * Initiate a remote function call.
     *
     * A remote function call is accomplished by following steps:
     * 1. Caller (CommServer) sends a message with the function name and
     *    arguments to Callee (CommClient), and waits for Callee's reply.
     * 2. Callee executes the function request by Caller, and send back the
     *    function name and returns to Caller.
     * 3. Caller receives the reply from Callee, check the function name that
     *    comes with it. Then Caller processes the rest of of the reply.
     *
     * The return of the function is sent back in the form of message.
     *
     * @param[in]   func_name   function name
     * @param[in]   sim_data    pointer to simulation data. NULL if the function
     *                          to call does not use simulation data.
     * @param[in]   args        input arguments
     */
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
        receive_msg(); // wait for return
        // make sure that we did get the return we want
        std::string reply;
        read_msg(reply);
        CHECK_EQ(reply, func_name);
    }

private:
    int port_; // listening port
    Acceptor acceptor_;
};

/**
 * Communicator on the client (child processes) side.
 */
class CommClient : public Communicator {
    using Resolver = boost::asio::ip::tcp::resolver;
    using TCP_Query = boost::asio::ip::tcp::resolver::query;
    using TCP_Iterator = boost::asio::ip::tcp::resolver::iterator;

public:
    CommClient(const std::string& host, const int port_no);

    ~CommClient() {}

protected:
    /**
     * Try to connect to server for at most MAX_ATTEMPTS times.
     */
    virtual bool establish_connection() override;

    std::string host_; // Address of server
    int port_; // port of server
    Resolver resolver_;
    static const int MAX_ATTEMPTS;
};

}} // simulator::communication
