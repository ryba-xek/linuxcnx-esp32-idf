
#ifndef ESP32_HOME_CTL_ETH_ESPIDF_ETH_H
#define ESP32_HOME_CTL_ETH_ESPIDF_ETH_H

#include <esp_err.h>

#define CONFIG_EXAMPLE_CONNECT_ETHERNET true
#define CONFIG_EXAMPLE_ETH_MDC_GPIO 23
#define CONFIG_EXAMPLE_ETH_MDIO_GPIO 18
#define CONFIG_EXAMPLE_ETH_PHY_RST_GPIO 16
#define CONFIG_EXAMPLE_ETH_PHY_ADDR 1
#define CONFIG_EXAMPLE_IP_ADDR        "192.168.96.54"
#define CONFIG_EXAMPLE_NETMASK_ADDR   "192.168.96.1"
#define CONFIG_EXAMPLE_GW_ADDR        "255.255.255.0"

#define CONFIG_EXAMPLE_CONNECT_WIFI false
#define CONFIG_EXAMPLE_WIFI_SSID "wifi ssid"
#define CONFIG_EXAMPLE_WIFI_PASSWORD ""

esp_err_t example_connect(void);

#endif //ESP32_HOME_CTL_ETH_ESPIDF_ETH_H
