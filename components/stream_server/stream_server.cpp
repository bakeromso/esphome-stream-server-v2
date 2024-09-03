/* Copyright (C) 2020-2022 Oxan van Leeuwen
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "stream_server.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/util.h"

#include "esphome/components/network/util.h"
#include "esphome/components/socket/socket.h"

static const char *TAG = "streamserver";

using namespace esphome;

void StreamServerComponent::setup()
{
    ESP_LOGCONFIG(TAG, "Setting up stream server...");

    struct sockaddr_in bind_addr = {};

    bind_addr.sin_len = sizeof(struct sockaddr_in);
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(this->port_);
    bind_addr.sin_addr.s_addr = ESPHOME_INADDR_ANY;
    memset(bind_addr.sin_zero, 0, sizeof(bind_addr.sin_zero)); // Zero-initialize sin_zero

    this->socket_ = socket::socket(AF_INET, SOCK_STREAM, PF_INET);

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 30000; // ESPHome recommends 20-30 ms max for timeouts

    this->socket_->setsockopt(SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout));

    this->socket_->bind(reinterpret_cast<struct sockaddr *>(&bind_addr), sizeof(struct sockaddr_in));
    this->socket_->listen(8);

    // Define and populate ip_str
    std::string ip_str;
    for (auto &ip : network::get_ip_addresses())
    {
        if (ip.is_set())
            ip_str += " " + ip.str();
    }

    ESP_LOGCONFIG(TAG, "Stream Server:");
    ESP_LOGCONFIG(TAG, "  Address:%s", ip_str.c_str());
    ESP_LOGCONFIG(TAG, "  Port: %u", this->port_);
}

void StreamServerComponent::loop()
{
    this->accept();
    this->read();
    this->write();
    this->cleanup();
}

void StreamServerComponent::accept()
{
    struct sockaddr_in client_addr;
    socklen_t client_addrlen = sizeof(struct sockaddr_in);
    std::unique_ptr<socket::Socket> socket = this->socket_->accept(reinterpret_cast<struct sockaddr *>(&client_addr), &client_addrlen);
    if (!socket)
    {
        // ESP_LOGE(TAG, "Failed to accept new client: %s", strerror(errno));
        return;
    }

    socket->setblocking(false);
    std::string identifier = socket->getpeername();
    this->clients_.emplace_back(std::move(socket), identifier);
    ESP_LOGD(TAG, "New client connected from %s", identifier.c_str());
}

void StreamServerComponent::cleanup()
{
    // Properly close and remove disconnected clients
    for (auto &client : this->clients_)
    {
        if (client.socket && client.disconnected)
        {
            client.socket->shutdown(SHUT_RDWR);
            client.socket->close();
        }
    }
    this->clients_.erase(
        std::remove_if(this->clients_.begin(), this->clients_.end(),
                       [](const Client &client)
                       { return client.disconnected; }),
        this->clients_.end());
}

void StreamServerComponent::read()
{
    int len;
    while ((len = this->stream_->available()) > 0)
    {
        char buf[128];
        len = std::min(len, 128);
        this->stream_->read_array(reinterpret_cast<uint8_t *>(buf), len);
        if (len > 0)
        {
            for (const Client &client : this->clients_)
            {
                ssize_t written = client.socket->write(buf, len);
                if (written < len)
                {
                    ESP_LOGW(TAG, "Failed to write all data to client %s", client.identifier.c_str());
                }
            }
        }
        else
        {
            ESP_LOGW(TAG, "No data read from stream or buffer overflow");
        }
    }
}

void StreamServerComponent::write()
{
    uint8_t buf[128];
    ssize_t len;
    for (Client &client : this->clients_)
    {
        while ((len = client.socket->read(&buf, sizeof(buf))) > 0)
        {
            this->stream_->write_array(buf, len);
        }
        if (len == 0)
        {
            ESP_LOGD(TAG, "Client %s disconnected", client.identifier.c_str());
            client.disconnected = true;
            continue;
        }
    }
}

void StreamServerComponent::dump_config()
{
    ESP_LOGCONFIG(TAG, "Stream Server:");
    std::string ip_str = "";
    for (auto &ip : network::get_ip_addresses())
    {
        if (ip.is_set())
            ip_str += " " + ip.str();
    }
    ESP_LOGCONFIG(TAG, "  Address:%s", ip_str.c_str());
    ESP_LOGCONFIG(TAG, "  Port: %u", this->port_);
}

void StreamServerComponent::on_shutdown()
{
    for (const Client &client : this->clients_)
        client.socket->shutdown(SHUT_RDWR);
}

StreamServerComponent::Client::Client(std::unique_ptr<esphome::socket::Socket> socket, std::string identifier)
    : socket(std::move(socket)), identifier{identifier}
{
}

void set_ip_address(const esphome::network::IPAddress &ip_address)
{
    this->ip_address_ = ip_address;
}
