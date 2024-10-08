# Copyright (C) 2021-2022 Oxan van Leeuwen
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID, CONF_PORT, CONF_IP_ADDRESS

# ESPHome doesn't know the Stream abstraction yet, so hardcode to use a UART for now.

AUTO_LOAD = ["socket"]

DEPENDENCIES = ["uart", "network"]

MULTI_CONF = True

ns = cg.global_ns
StreamServerComponent = ns.class_("StreamServerComponent", cg.Component)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(StreamServerComponent),
            cv.Optional(CONF_PORT): cv.port,
            cv.Optional(CONF_IP_ADDRESS): cv.ipv4,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    if CONF_PORT in config:
        cg.add(var.set_port(config[CONF_PORT]))
    if CONF_IP_ADDRESS in config:
        ip_address = config[CONF_IP_ADDRESS]
        ip_address_str = str(ip_address)  # Convert IPAddress object to string
        octets = ip_address_str.split('.')  # Split the string into octets
        ip_address_expr = cg.RawExpression(
            f"IPAddress({octets[0]}, {octets[1]}, {octets[2]}, {octets[3]})"
        )
        cg.add(var.set_ip_address(ip_address_expr))

    yield cg.register_component(var, config)
    yield uart.register_uart_device(var, config)
