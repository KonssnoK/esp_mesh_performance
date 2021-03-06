/* Mesh Internal Communication Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_mesh_internal.h"
#include "mesh_light.h"
#include "nvs_flash.h"

/*******************************************************
 *                Macros
 *******************************************************/
 //#define MESH_P2P_TOS_OFF

 /*******************************************************
  *                Constants
  *******************************************************/
#define RX_SIZE          (1500)
#define TX_SIZE          (1460)

  /*******************************************************
   *                Variable Definitions
   *******************************************************/
static const char *MESH_TAG = "mesh_client";
static const uint8_t MESH_ID[6] = { 0x77, 0x77, 0x77, 0x77, 0x77, 0x77 };
static uint8_t tx_buf[TX_SIZE] = { 0, };
static uint8_t rx_buf[RX_SIZE] = { 0, };
static bool is_running = true;
static bool is_mesh_connected = false;
static mesh_addr_t mesh_parent_addr = { 0 };
static int mesh_layer = -1;

//#pragma region COMMON

#define MAC_MATCH(x, y) ((x[0] == y[0]) && (x[1] == y[1]) && (x[2] == y[2]) && (x[3] == y[3]) && (x[4] == y[4]) && (x[5] == y[5]))
#define MAC6_SIZE (6)

#define MAC_CODE(x) ((((uint16_t)x[4]) << 8) | x[5])

#define PKT_ID_COUNTER 0x3A
#define PKT_ID_COUNTER_ANS	0xCA

typedef struct payload_head_ {
	uint32_t ssc;
	uint8_t pkt_id;
} payload_head_t;

typedef struct payload_node_ {
	payload_head_t head;
	//Spefic payload
	uint8_t layer;				//Mesh library obtained layer
	uint8_t parent[MAC6_SIZE];	//
	int8_t rssi;				//RSSI of the node to the first parent
} payload_node_t;

typedef struct payload_root_ {
	payload_head_t head;
	//Spefic payload
	int8_t txPwr;	//TX Power to set
} payload_root_t;

//#pragma endregion


/*******************************************************
 *                Function Declarations
 *******************************************************/

static uint8_t _my_macSTA[MAC6_SIZE] = { 0 };
static uint8_t _my_macAP[MAC6_SIZE] = { 0 };
static int8_t _wifi_tx_pwr = 0;

 /*******************************************************
  *                Function Definitions
  *******************************************************/
void esp_mesh_p2p_tx_main(void *arg)
{
	is_running = true;
	while (is_running) {

		//We are a node, therefore we just send replies to the root

		/* non-root do nothing but print */
		if (!esp_mesh_is_root()) {
			ESP_LOGI(MESH_TAG, "layer:%d, rtableSize:%d, %s", mesh_layer,
				esp_mesh_get_routing_table_size(),
				(is_mesh_connected && esp_mesh_is_root()) ? "ROOT" : is_mesh_connected ? "NODE" : "DISCONNECT");
			vTaskDelay(10 * 10000 / portTICK_RATE_MS);
			continue;
		}
	}
	vTaskDelete(NULL);
}

void esp_mesh_p2p_rx_main(void *arg)
{
	int recv_count = 0;
	esp_err_t err;
	mesh_addr_t from;
	mesh_data_t dataRX;
	int flag = 0;
	dataRX.data = rx_buf;
	dataRX.size = RX_SIZE;

	wifi_ap_record_t apr;

	mesh_data_t dataTX;
	dataTX.data = tx_buf;
	dataTX.size = sizeof(tx_buf);
	dataTX.proto = MESH_PROTO_BIN;

	payload_root_t* pr;
	payload_node_t* pt;

	is_running = true;
	while (is_running) {
		dataRX.size = RX_SIZE;
		err = esp_mesh_recv(&from, &dataRX, portMAX_DELAY, &flag, NULL, 0);
		if (err != ESP_OK || !dataRX.size) {
			ESP_LOGE(MESH_TAG, "RX err:0x%x, size:%d", err, dataRX.size);
			continue;
		}

		if (MAC_MATCH(_my_macSTA, from.addr)) {
			//Let's ignore ourself
			continue;
		}

		recv_count++;

		/* extract send count */
		if (dataRX.size < sizeof(payload_root_t)) {
			ESP_LOGW(MESH_TAG, "Packet too short! %d", dataRX.size);
			continue;
		}

		pr = (payload_root_t*)dataRX.data;
		if (pr->head.pkt_id != PKT_ID_COUNTER) {
			ESP_LOGW(MESH_TAG, "Received unexpected packet %02X", pr->head.pkt_id);
			continue;
		}

		uint32_t ssc = ntohl(pr->head.ssc);
		int8_t txpwr = pr->txPwr;
		ESP_LOGI(MESH_TAG, "RECV SSC %d from "MACSTR, ssc, MAC2STR(from.addr));

		//Use payload
		if (txpwr != _wifi_tx_pwr) {
			//Change TX power as requested from root
			ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(txpwr));
			_wifi_tx_pwr = txpwr;
		}
		//End

		pt = (payload_node_t*)dataTX.data;
		pt->head.pkt_id = PKT_ID_COUNTER_ANS;
		pt->head.ssc = htonl(ssc);
		pt->layer = (uint8_t)mesh_layer;
		memcpy(pt->parent, mesh_parent_addr.addr, MAC6_SIZE);
		ESP_ERROR_CHECK(esp_wifi_sta_get_ap_info(&apr)); //Result also RSSI data
		pt->rssi = apr.rssi;
		
		dataTX.size = sizeof(payload_node_t);

		//Answer back to the root
		err = esp_mesh_send(&from, &dataTX, MESH_DATA_P2P, NULL, 0);
		if (err != ESP_OK) {
			ESP_LOGE(MESH_TAG, "TX err:0x%x", err);
		}
	}
	vTaskDelete(NULL);
}

esp_err_t esp_mesh_comm_p2p_start(void)
{
	static bool is_comm_p2p_started = false;
	if (!is_comm_p2p_started) {
		is_comm_p2p_started = true;

		ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, _my_macSTA));
		ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_AP, _my_macAP));

		xTaskCreate(esp_mesh_p2p_tx_main, "MPTX", 3072, NULL, 5, NULL);
		xTaskCreate(esp_mesh_p2p_rx_main, "MPRX", 3072, NULL, 5, NULL);
	}
	return ESP_OK;
}

void mesh_event_handler(mesh_event_t event)
{
	mesh_addr_t id = { 0, };
	static uint8_t last_layer = 0;
	ESP_LOGD(MESH_TAG, "esp_event_handler:%d", event.id);

	switch (event.id) {
	case MESH_EVENT_STARTED:
		esp_mesh_get_id(&id);
		ESP_LOGI(MESH_TAG, "<MESH_EVENT_STARTED>ID:"MACSTR"", MAC2STR(id.addr));
		is_mesh_connected = false;
		mesh_layer = esp_mesh_get_layer();
		break;
	case MESH_EVENT_STOPPED:
		ESP_LOGI(MESH_TAG, "<MESH_EVENT_STOPPED>");
		is_mesh_connected = false;
		mesh_layer = esp_mesh_get_layer();
		break;
	case MESH_EVENT_CHILD_CONNECTED:
		ESP_LOGI(MESH_TAG, "<MESH_EVENT_CHILD_CONNECTED>aid:%d, "MACSTR"",
			event.info.child_connected.aid,
			MAC2STR(event.info.child_connected.mac));
		break;
	case MESH_EVENT_CHILD_DISCONNECTED:
		ESP_LOGI(MESH_TAG, "<MESH_EVENT_CHILD_DISCONNECTED>aid:%d, "MACSTR"",
			event.info.child_disconnected.aid,
			MAC2STR(event.info.child_disconnected.mac));
		break;
	case MESH_EVENT_ROUTING_TABLE_ADD:
		ESP_LOGW(MESH_TAG, "<MESH_EVENT_ROUTING_TABLE_ADD>add %d, new:%d",
			event.info.routing_table.rt_size_change,
			event.info.routing_table.rt_size_new);
		break;
	case MESH_EVENT_ROUTING_TABLE_REMOVE:
		ESP_LOGW(MESH_TAG, "<MESH_EVENT_ROUTING_TABLE_REMOVE>remove %d, new:%d",
			event.info.routing_table.rt_size_change,
			event.info.routing_table.rt_size_new);
		break;
	case MESH_EVENT_NO_PARENT_FOUND:
		ESP_LOGI(MESH_TAG, "<MESH_EVENT_NO_PARENT_FOUND>scan times:%d",
			event.info.no_parent.scan_times);
		/* TODO handler for the failure */
		break;
	case MESH_EVENT_PARENT_CONNECTED:
		esp_mesh_get_id(&id);
		mesh_layer = event.info.connected.self_layer;
		memcpy(&mesh_parent_addr.addr, event.info.connected.connected.bssid, 6);
		ESP_LOGI(MESH_TAG,
			"<MESH_EVENT_PARENT_CONNECTED>layer:%d-->%d, parent:"MACSTR"%s, ID:"MACSTR"",
			last_layer, mesh_layer, MAC2STR(mesh_parent_addr.addr),
			esp_mesh_is_root() ? "<ROOT>" :
			(mesh_layer == 2) ? "<layer2>" : "", MAC2STR(id.addr));
		last_layer = mesh_layer;
		
		is_mesh_connected = true;
		
		esp_mesh_comm_p2p_start();
		break;
	case MESH_EVENT_PARENT_DISCONNECTED:
		ESP_LOGI(MESH_TAG,
			"<MESH_EVENT_PARENT_DISCONNECTED>reason:%d",
			event.info.disconnected.reason);
		is_mesh_connected = false;
		
		mesh_layer = esp_mesh_get_layer();
		break;
	case MESH_EVENT_LAYER_CHANGE:
		mesh_layer = event.info.layer_change.new_layer;
		ESP_LOGI(MESH_TAG, "<MESH_EVENT_LAYER_CHANGE>layer:%d-->%d%s",
			last_layer, mesh_layer,
			esp_mesh_is_root() ? "<ROOT>" :
			(mesh_layer == 2) ? "<layer2>" : "");
		last_layer = mesh_layer;
		break;
	case MESH_EVENT_ROOT_ADDRESS:
		ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_ADDRESS>root address:"MACSTR"",
			MAC2STR(event.info.root_addr.addr));
		break;
	case MESH_EVENT_ROOT_GOT_IP:
		/* root starts to connect to server */
		ESP_LOGI(MESH_TAG,
			"<MESH_EVENT_ROOT_GOT_IP>sta ip: " IPSTR ", mask: " IPSTR ", gw: " IPSTR,
			IP2STR(&event.info.got_ip.ip_info.ip),
			IP2STR(&event.info.got_ip.ip_info.netmask),
			IP2STR(&event.info.got_ip.ip_info.gw));
		break;
	case MESH_EVENT_ROOT_LOST_IP:
		ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_LOST_IP>");
		break;
	case MESH_EVENT_VOTE_STARTED:
		ESP_LOGI(MESH_TAG,
			"<MESH_EVENT_VOTE_STARTED>attempts:%d, reason:%d, rc_addr:"MACSTR"",
			event.info.vote_started.attempts,
			event.info.vote_started.reason,
			MAC2STR(event.info.vote_started.rc_addr.addr));
		break;
	case MESH_EVENT_VOTE_STOPPED:
		ESP_LOGI(MESH_TAG, "<MESH_EVENT_VOTE_STOPPED>");
		break;
	case MESH_EVENT_ROOT_SWITCH_REQ:
		ESP_LOGI(MESH_TAG,
			"<MESH_EVENT_ROOT_SWITCH_REQ>reason:%d, rc_addr:"MACSTR"",
			event.info.switch_req.reason,
			MAC2STR(event.info.switch_req.rc_addr.addr));
		break;
	case MESH_EVENT_ROOT_SWITCH_ACK:
		/* new root */
		mesh_layer = esp_mesh_get_layer();
		esp_mesh_get_parent_bssid(&mesh_parent_addr);
		ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_SWITCH_ACK>layer:%d, parent:"MACSTR"", mesh_layer, MAC2STR(mesh_parent_addr.addr));
		break;
	case MESH_EVENT_TODS_STATE:
		ESP_LOGI(MESH_TAG, "<MESH_EVENT_TODS_REACHABLE>state:%d",
			event.info.toDS_state);
		break;
	case MESH_EVENT_ROOT_FIXED:
		ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_FIXED>%s",
			event.info.root_fixed.is_fixed ? "fixed" : "not fixed");
		break;
	case MESH_EVENT_ROOT_ASKED_YIELD:
		ESP_LOGI(MESH_TAG,
			"<MESH_EVENT_ROOT_ASKED_YIELD>"MACSTR", rssi:%d, capacity:%d",
			MAC2STR(event.info.root_conflict.addr),
			event.info.root_conflict.rssi,
			event.info.root_conflict.capacity);
		break;
	case MESH_EVENT_CHANNEL_SWITCH:
		ESP_LOGI(MESH_TAG, "<MESH_EVENT_CHANNEL_SWITCH>new channel:%d", event.info.channel_switch.channel);
		break;
	case MESH_EVENT_SCAN_DONE:
		ESP_LOGI(MESH_TAG, "<MESH_EVENT_SCAN_DONE>number:%d",
			event.info.scan_done.number);
		break;
	case MESH_EVENT_NETWORK_STATE:
		ESP_LOGI(MESH_TAG, "<MESH_EVENT_NETWORK_STATE>is_rootless:%d",
			event.info.network_state.is_rootless);
		break;
	case MESH_EVENT_STOP_RECONNECTION:
		ESP_LOGI(MESH_TAG, "<MESH_EVENT_STOP_RECONNECTION>");
		break;
	case MESH_EVENT_FIND_NETWORK:
		ESP_LOGI(MESH_TAG, "<MESH_EVENT_FIND_NETWORK>new channel:%d, router BSSID:"MACSTR"",
			event.info.find_network.channel, MAC2STR(event.info.find_network.router_bssid));
		break;
	case MESH_EVENT_ROUTER_SWITCH:
		ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROUTER_SWITCH>new router:%s, channel:%d, "MACSTR"",
			event.info.router_switch.ssid, event.info.router_switch.channel, MAC2STR(event.info.router_switch.bssid));
		break;
	default:
		ESP_LOGI(MESH_TAG, "unknown id:%d", event.id);
		break;
	}
}

void app_main(void)
{
	ESP_ERROR_CHECK(mesh_light_init());
	ESP_ERROR_CHECK(nvs_flash_init());
	/*  tcpip initialization */
	tcpip_adapter_init();
	/* for mesh
	 * stop DHCP server on softAP interface by default
	 * stop DHCP client on station interface by default
	 * */
	ESP_ERROR_CHECK(tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP));
	ESP_ERROR_CHECK(tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_STA));
#if 0
	/* static ip settings */
	tcpip_adapter_ip_info_t sta_ip;
	sta_ip.ip.addr = ipaddr_addr("192.168.1.102");
	sta_ip.gw.addr = ipaddr_addr("192.168.1.1");
	sta_ip.netmask.addr = ipaddr_addr("255.255.255.0");
	tcpip_adapter_set_ip_info(WIFI_IF_STA, &sta_ip);
#endif
	/*  wifi initialization */
	ESP_ERROR_CHECK(esp_event_loop_init(NULL, NULL));
	wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&config));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
	ESP_ERROR_CHECK(esp_wifi_start());

	////Reduce WIFI TX power to the minimum
	//int8_t pwr;
	//ESP_ERROR_CHECK(esp_wifi_get_max_tx_power(&pwr));
	//ESP_LOGI(MESH_TAG, "WIFI power currently set to %d.", pwr);
	//pwr = 7;
	//ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(pwr));

	/*  mesh initialization */
	ESP_ERROR_CHECK(esp_mesh_init());
	ESP_ERROR_CHECK(esp_mesh_set_max_layer(CONFIG_MESH_MAX_LAYER));
	//ESP_ERROR_CHECK(esp_mesh_set_vote_percentage(1));
	ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(10));

	//Fix the root node
	ESP_ERROR_CHECK(esp_mesh_fix_root(1));
	//ESP_ERROR_CHECK(esp_mesh_set_type(MESH_LEAF));

	mesh_cfg_t cfg = MESH_INIT_CONFIG_DEFAULT();
	/* mesh ID */
	memcpy((uint8_t *)&cfg.mesh_id, MESH_ID, 6);
	/* mesh event callback */
	cfg.event_cb = &mesh_event_handler;
	/* router */
	cfg.channel = CONFIG_MESH_CHANNEL;
	cfg.allow_channel_switch = true; //Allows client to change channel because the root will take the channel of the router!

	//Do not allow devices to connect to the router
	cfg.router.ssid_len = strlen(CONFIG_MESH_ROUTER_SSID);
	memcpy((uint8_t *)&cfg.router.ssid, CONFIG_MESH_ROUTER_SSID, cfg.router.ssid_len);
	memcpy((uint8_t *)&cfg.router.password, CONFIG_MESH_ROUTER_PASSWD,
		strlen(CONFIG_MESH_ROUTER_PASSWD));

	/* mesh softAP */
	ESP_ERROR_CHECK(esp_mesh_set_ap_authmode(CONFIG_MESH_AP_AUTHMODE));
	cfg.mesh_ap.max_connection = CONFIG_MESH_AP_CONNECTIONS;
	memcpy((uint8_t *)&cfg.mesh_ap.password, CONFIG_MESH_AP_PASSWD,
		strlen(CONFIG_MESH_AP_PASSWD));
	ESP_ERROR_CHECK(esp_mesh_set_config(&cfg));

	/* mesh start */
	ESP_ERROR_CHECK(esp_mesh_start());
	ESP_LOGI(MESH_TAG, "mesh starts successfully, heap:%d, %s\n", esp_get_free_heap_size(),
		esp_mesh_is_root_fixed() ? "root fixed" : "root not fixed");
}
