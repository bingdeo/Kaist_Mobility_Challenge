#!/usr/bin/env bash
set -euo pipefail

# This script installs FastRTPS RMW on the host and generates FastDDS XML profiles.
# It writes the same XML to:
#   1) /tmp/fastdds.xml   (handy for docker -v /tmp/fastdds.xml:/tmp/fastdds.xml:ro)
#   2) $HOME/fastdds.xml  (user-home outermost location: /home/<user>/fastdds.xml)

XML_TMP="/tmp/fastdds.xml"
XML_HOME="${HOME}/fastdds.xml"

# 1) Install FastRTPS RMW (host)
sudo apt-get update
sudo apt-get install -y ros-foxy-rmw-fastrtps-cpp

# 2) Generate XML in /tmp
cat >"${XML_TMP}" <<'XML'
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>udp_transport</transport_id>
            <type>UDPv4</type>
        </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="udp_participant_profile" is_default_profile="true">
        <rtps>
            <useBuiltinTransports>false</useBuiltinTransports>
            <userTransports>
                <transport_id>udp_transport</transport_id>
            </userTransports>
        </rtps>
    </participant>
</profiles>
XML
chmod 0644 "${XML_TMP}"

# 3) Copy the same XML to $HOME
install -m 0644 "${XML_TMP}" "${XML_HOME}"

echo "DONE"
echo "  XML (tmp) : ${XML_TMP}"
echo "  XML (home): ${XML_HOME}"