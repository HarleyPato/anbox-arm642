/*
 * Copyright (C) 2016 Simon Fels <morphis@gravedo.de>
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 3, as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranties of
 * MERCHANTABILITY, SATISFACTORY QUALITY, or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <string>

#include "anbox/logger.h"
#include "anbox/network/socket_messenger.h"
#include "anbox/graphics/opengles_message_processor.h"
#include "anbox/qemu/pipe_connection_creator.h"
#include "anbox/qemu/boot_properties_message_processor.h"
#include "anbox/qemu/null_message_processor.h"
#include "anbox/qemu/hwcontrol_message_processor.h"
#include "anbox/qemu/sensors_message_processor.h"
#include "anbox/qemu/camera_message_processor.h"
#include "anbox/qemu/fingerprint_message_processor.h"
#include "anbox/qemu/gsm_message_processor.h"
#include "anbox/qemu/bootanimation_message_processor.h"

namespace ba = boost::asio;

namespace anbox {
namespace qemu {
PipeConnectionCreator::PipeConnectionCreator(const std::shared_ptr<Runtime> &rt,
                                             const std::string &renderer_socket_path,
                                             const std::string &boot_animation_icon_path) :
    runtime_(rt),
    next_connection_id_(0),
    connections_(std::make_shared<network::Connections<network::SocketConnection>>()),
    renderer_socket_path_(renderer_socket_path),
    boot_animation_icon_path_(boot_animation_icon_path) {
}

PipeConnectionCreator::~PipeConnectionCreator() {
}

void PipeConnectionCreator::create_connection_for(
        std::shared_ptr<boost::asio::local::stream_protocol::socket> const& socket) {

    auto const messenger = std::make_shared<network::SocketMessenger>(socket);
    const auto type = identify_client(messenger);
    auto const processor = create_processor(type, messenger);
    if (!processor)
        BOOST_THROW_EXCEPTION(std::runtime_error("Unhandled client type"));

    auto const& connection = std::make_shared<network::SocketConnection>(
                messenger, messenger, next_id(), connections_, processor);
    connections_->add(connection);
    connection->read_next_message();
}

PipeConnectionCreator::client_type PipeConnectionCreator::identify_client(
        std::shared_ptr<network::SocketMessenger> const& messenger) {

    // The client will identify itself as first thing by writing a string
    // in the format 'pipe:<name>[:<arguments>]\0' to the channel.
    std::vector<char> buffer;
    for (;;) {
        unsigned char byte[1] = { 0 };
        const auto err = messenger->receive_msg(ba::buffer(byte, 1));
        if (err)
            break;
        buffer.push_back(byte[0]);
        if (byte[0] == 0x0)
            break;
    }

    std::string identifier_and_args = buffer.data();

    DEBUG("identifier %s", identifier_and_args);

    if (utils::string_starts_with(identifier_and_args, "pipe:opengles"))
        return client_type::opengles;
    // Even if 'boot-properties' is an argument to the service 'qemud' here we
    // take this as a own service instance as that is what it is.
    else if (utils::string_starts_with(identifier_and_args, "pipe:qemud:boot-properties"))
        return client_type::qemud_boot_properties;
    else if (utils::string_starts_with(identifier_and_args, "pipe:qemud:hw-control"))
        return client_type::qemud_hw_control;
    else if (utils::string_starts_with(identifier_and_args, "pipe:qemud:sensors"))
        return client_type::qemud_sensors;
    else if (utils::string_starts_with(identifier_and_args, "pipe:qemud:camera"))
        return client_type::qemud_camera;
    else if (utils::string_starts_with(identifier_and_args, "pipe:qemud:fingerprintlisten"))
        return client_type::qemud_fingerprint;
    else if (utils::string_starts_with(identifier_and_args, "pipe:qemud:gsm"))
        return client_type::qemud_gsm;
    else if (utils::string_starts_with(identifier_and_args, "pipe:anbox:bootanimation"))
        return client_type::bootanimation;

    return client_type::invalid;
}

std::shared_ptr<network::MessageProcessor> PipeConnectionCreator::create_processor(const client_type &type, const std::shared_ptr<network::SocketMessenger> &messenger) {
    if (type == client_type::opengles)
        return std::make_shared<graphics::OpenGlesMessageProcessor>(renderer_socket_path_, runtime_, messenger);
    else if (type == client_type::qemud_boot_properties)
        return std::make_shared<qemu::BootPropertiesMessageProcessor>(messenger);
    else if (type == client_type::qemud_hw_control)
        return std::make_shared<qemu::HwControlMessageProcessor>(messenger);
    else if (type == client_type::qemud_sensors)
        return std::make_shared<qemu::SensorsMessageProcessor>(messenger);
    else if (type == client_type::qemud_camera)
        return std::make_shared<qemu::CameraMessageProcessor>(messenger);
    else if (type == client_type::qemud_fingerprint)
        return std::make_shared<qemu::FingerprintMessageProcessor>(messenger);
    else if (type == client_type::qemud_gsm)
        return std::make_shared<qemu::GsmMessageProcessor>(messenger);
    else if (type == client_type::bootanimation)
        return std::make_shared<qemu::BootAnimationMessageProcessor>(messenger, boot_animation_icon_path_);

    return std::make_shared<qemu::NullMessageProcessor>();
}

int PipeConnectionCreator::next_id()
{
    return next_connection_id_.fetch_add(1);
}
} // namespace qemu
} // namespace anbox