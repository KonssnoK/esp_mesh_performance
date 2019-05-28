/* Mesh Internal Communication Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
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
#define WIFI_TX_PWR		20	//LOOK BELOW
/* - [78, 127]: level0
 * - [76, 77] : level1
 * - [74, 75] : level2
 * - [68, 73] : level3
 * - [60, 67] : level4
 * - [52, 59] : level5
 * - [44, 51] : level5 - 2dBm
 * - [34, 43] : level5 - 4.5dBm
 * - [28, 33] : level5 - 6dBm
 * - [20, 27] : level5 - 8dBm
 * - [ 8, 19] : level5 - 11dBm
 * - [-128, 7] : level5 - 14dBm 
 */

 /*******************************************************
  *                Constants
  *******************************************************/
#define RX_SIZE          (1500)
#define TX_SIZE          (1460)

  /*******************************************************
   *                Variable Definitions
   *******************************************************/
static const char *MESH_TAG = "mesh_root";
static const uint8_t MESH_ID[6] = { 0x77, 0x77, 0x77, 0x77, 0x77, 0x77 };
static uint8_t tx_buf[TX_SIZE] = { 0, };
static uint8_t rx_buf[RX_SIZE] = { 0, };
static bool is_running = true;
static bool is_mesh_connected = false;
static mesh_addr_t mesh_parent_addr;
static int mesh_layer = -1;

//#pragma region COMMON

#define MAC_MATCH(x, y) ((x[0] == y[0]) && (x[1] == y[1]) && (x[2] == y[2]) && (x[3] == y[3]) && (x[4] == y[4]) && (x[5] == y[5]))
#define MAC6_SIZE (6)

#define MAC_CODE(x) ((((uint16_t)x[4]) << 8) | x[5])

#define PKT_ID_COUNTER 0x3A
#define PKT_ID_COUNTER_ANS	0xCA

typedef struct {
	uint32_t ssc;
	uint8_t pkt_id;
} payload_head_t;

typedef struct {
	payload_head_t head;
	//Spefic payload
	uint8_t layer;				//Mesh library obtained layer
	uint8_t parent[MAC6_SIZE];	//
	int8_t rssi;				//RSSI of the node to the first parent
} payload_node_t;

typedef struct {
	payload_head_t head;
	//Spefic payload
	int8_t txPwr;	//TX Power to set
} payload_root_t;

//#pragma endregion

typedef struct rtt {
	uint64_t start;
	uint32_t id;
	uint16_t mac_coded;
	bool used;
} rtt_t;

#define MAX_RTT_SENT 100
static rtt_t _rounders[MAX_RTT_SENT];
static uint32_t _rounder_write = 0;

typedef struct {
	uint8_t layer;
	uint16_t mac_coded;
	uint16_t parent_coded;
	int8_t rssi;
	bool exists;
} children_t;

#define MAX_DEVICES 20
children_t _my_network[MAX_DEVICES] = { 0 };
char _the_string[512] = { 0 };


/*******************************************************
 *                Function Declarations
 *******************************************************/

static uint8_t _my_macSTA[MAC6_SIZE] = { 0 };
static uint8_t _my_macAP[MAC6_SIZE] = { 0 };
static uint32_t _ssc_sent = 0;

 /*******************************************************
  *                Function Definitions
  *******************************************************/
void esp_mesh_p2p_tx_main(void *arg)
{
	int i;
	esp_err_t err;
	mesh_addr_t route_table[CONFIG_MESH_ROUTE_TABLE_SIZE];
	int route_table_size = 0;
	//TX packet
	mesh_data_t dataTX;
	dataTX.data = tx_buf;
	dataTX.size = sizeof(tx_buf);
	dataTX.proto = MESH_PROTO_BIN;

	payload_root_t *pt;
	rtt_t * r;
	uint64_t lastDrawnUS = esp_timer_get_time(); 
	

	is_running = true;
	while (is_running) {

		//Get all the nodes
		esp_mesh_get_routing_table((mesh_addr_t *)&route_table,
			CONFIG_MESH_ROUTE_TABLE_SIZE * 6, &route_table_size);

		pt = (payload_root_t*)tx_buf;
		pt->head.pkt_id = PKT_ID_COUNTER;

		//For each connected node
		for (i = 0; i < route_table_size; i++) {

			if (MAC_MATCH(_my_macSTA, route_table[i].addr)) {
				//Let's ignore ourself
				continue;
			}

			_ssc_sent++;
			pt->head.ssc = htonl(_ssc_sent);
			pt->txPwr = WIFI_TX_PWR;
			

			err = esp_mesh_send(&route_table[i], &dataTX, MESH_DATA_P2P, NULL, 0);
			if (err) {
				ESP_LOGE(MESH_TAG,
					"TX [L:%d]parent:"MACSTR" to "MACSTR", [err:0x%x]",
					mesh_layer, 
					MAC2STR(mesh_parent_addr.addr),
					MAC2STR(route_table[i].addr), 
					err);
			}
			else {
				//Add packet to the list of awaiting answers
				r = &(_rounders[_rounder_write]);

				if (r->used) {
					ESP_LOGE(MESH_TAG, "Overwriting pkt %d for %X after %dms", 
						r->id,
						r->mac_coded,
						(uint32_t)((esp_timer_get_time() - r->start) / 1000));
				}
				r->id = _ssc_sent;
				r->start = esp_timer_get_time();
				r->used = true;
				r->mac_coded = MAC_CODE(route_table[i].addr);
				_rounder_write++;
				if (_rounder_write >= MAX_RTT_SENT)
					_rounder_write = 0;
			}

		}

		uint16_t idx = 0;
		//Do ancillary jobs
		if ((esp_timer_get_time() - lastDrawnUS) > 5000000U) {//Every 5 seconds
			//Draw the network
			ESP_LOGW(MESH_TAG, "Layer 0 means unknown! ME(%X)", MAC_CODE(_my_macAP));
			for (int i = 0; i < CONFIG_MESH_MAX_LAYER; ++i) {

				idx = 0;

				for(int j = 0; j < MAX_DEVICES; ++j){
					//If this is the right layer and the node is active
					if(_my_network[j].exists && (_my_network[j].layer == i)){
						idx += sprintf(&_the_string[idx], "(%X<-%X[%d]) ", 
							_my_network[j].parent_coded, _my_network[j].mac_coded, _my_network[j].rssi);
					}
				}
				//Terminate string
				_the_string[idx] = 0;
				if(idx)
					ESP_LOGW(MESH_TAG, "layer %d %s", i, _the_string);
			}
			lastDrawnUS = esp_timer_get_time();
		}

		/* if route_table_size is less than 10, add delay to avoid watchdog in this task. */
		//if (route_table_size < 10) {
		vTaskDelay(2 * 1000 / portTICK_RATE_MS);
		//}
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

	payload_node_t *p;
	rtt_t* r;

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
		if (dataRX.size < sizeof(payload_node_t)) {
			ESP_LOGW(MESH_TAG, "Packet too short from %02X%02X! %d", 
				from.addr[4],
				from.addr[5],
				dataRX.size);
			continue;
		}

		p = (payload_node_t*)dataRX.data;
		if (p->head.pkt_id != PKT_ID_COUNTER_ANS) {
			ESP_LOGW(MESH_TAG, "Received unexpected packet %02X from %02X%02X",
				p->head.pkt_id,
				from.addr[4],
				from.addr[5]);
			continue;
		}

		int ssc = ntohl(p->head.ssc);
		for (int i = 0; i < MAX_RTT_SENT; ++i) {
			r = &_rounders[i];
			if (r->id == ssc) {
				ESP_LOGI(MESH_TAG, "%X: RTT %d. Elapsed %dms",
					MAC_CODE(from.addr),
					r->id,
					(uint32_t)((esp_timer_get_time() - r->start)/1000));
				r->used = false;
				break;
			}
		}

		//Go on with the packet
		for (int i = 0; i < MAX_DEVICES; ++i) {
			if (_my_network[i].exists && _my_network[i].mac_coded == MAC_CODE(from.addr)) {
				//Update parent and layer info
				_my_network[i].layer = p->layer;
				_my_network[i].parent_coded = MAC_CODE(p->parent);
				_my_network[i].rssi = p->rssi;
			}
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
		ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_AP , _my_macAP ));

		xTaskCreate(esp_mesh_p2p_tx_main, "MPTX", 3072, NULL, 5, NULL);
		xTaskCreate(esp_mesh_p2p_rx_main, "MPRX", 3072, NULL, 5, NULL);
	}
	return ESP_OK;
}

void mesh_event_handler(mesh_event_t event)
{
	mesh_addr_t id = { 0, };
	static uint8_t last_layer = 0;

	mesh_addr_t route_table[CONFIG_MESH_ROUTE_TABLE_SIZE];
	int route_table_size = 0;

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

		//Since the event does not give a list of the new devices we have to check ALL of them :(		
		esp_mesh_get_routing_table((mesh_addr_t *)&route_table,
			CONFIG_MESH_ROUTE_TABLE_SIZE * 6, &route_table_size);

		for (int j = 0; j < route_table_size; ++j) {
			//Add node, if needed
			int i;
			for (i = 0; i < MAX_DEVICES; ++i) {
				//We don't trust callback to be called only for new nodes. 
				//If we already have the device in the map, we use that one
				if (_my_network[i].mac_coded == MAC_CODE(route_table[j].addr)) {
					_my_network[i].exists = true;//Entry is active from here
					break;
				}
			}
			//If the device mac was not found in the list
			if (i >= MAX_DEVICES) {
				for (i = 0; i < MAX_DEVICES; ++i) {
					//Get the first free slot and set it up
					if (!_my_network[i].exists) {
						//Create entry
						_my_network[i].mac_coded = MAC_CODE(route_table[j].addr);
						_my_network[i].layer = 0;
						_my_network[i].exists = true;//Entry is active from here
						break;
					}
				}
			}
		}

		break;
	case MESH_EVENT_ROUTING_TABLE_REMOVE:
		ESP_LOGW(MESH_TAG, "<MESH_EVENT_ROUTING_TABLE_REMOVE>remove %d, new:%d",
			event.info.routing_table.rt_size_change,
			event.info.routing_table.rt_size_new);

		//Since the event does not give a list of the removed devices we have to check ALL of them :(
		esp_mesh_get_routing_table((mesh_addr_t *)&route_table,
			CONFIG_MESH_ROUTE_TABLE_SIZE * 6, &route_table_size);

		bool found = false;
		
		for (int i = 0; i < MAX_DEVICES; ++i) {
			//Remove any device that is not referenced in the routing table anymore
			found = false;

			if (!_my_network[i].exists)
				continue;//Skip inactive slots

			//Search for the mac in the routing table
			for (int j = 0; j < route_table_size; ++j) {
				if (_my_network[i].mac_coded == MAC_CODE(route_table[j].addr)) {
					found = true;
					break;//uniqueness is guaranteed in the add function
				}
			}
			if (found) {
				//Reset the slot
				_my_network[i].exists = false;
				_my_network[i].mac_coded = 0;
			}
		}

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
		if (esp_mesh_is_root()) {
			tcpip_adapter_dhcpc_start(TCPIP_ADAPTER_IF_STA);
		}
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

	//Reduce WIFI TX power to the minimum
	int8_t pwr;
	ESP_ERROR_CHECK(esp_wifi_get_max_tx_power(&pwr));
	ESP_LOGI(MESH_TAG, "WIFI power currently set to %d.", pwr);
	pwr = WIFI_TX_PWR;
	ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(pwr));

	/*  mesh initialization */
	ESP_ERROR_CHECK(esp_mesh_init());
	ESP_ERROR_CHECK(esp_mesh_set_max_layer(CONFIG_MESH_MAX_LAYER));
	//ESP_ERROR_CHECK(esp_mesh_set_vote_percentage(1));
	ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(10));

	//Fix the root node
	ESP_ERROR_CHECK(esp_mesh_fix_root(1));
	ESP_ERROR_CHECK(esp_mesh_set_type(MESH_ROOT));
	//wifi_config_t parent;
	//parent.ap.
	//ESP_ERROR_CHECK(esp_mesh_set_parent(&parent, , MESH_ROOT, MESH_ROOT_LAYER));

	mesh_cfg_t cfg = MESH_INIT_CONFIG_DEFAULT();
	/* mesh ID */
	memcpy((uint8_t *)&cfg.mesh_id, MESH_ID, 6);
	/* mesh event callback */
	cfg.event_cb = &mesh_event_handler;
	/* router */
	cfg.channel = CONFIG_MESH_CHANNEL;

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
