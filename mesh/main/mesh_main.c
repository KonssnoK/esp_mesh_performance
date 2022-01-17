/* Mesh Internal Communication Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_mesh_internal.h"
#include "mesh_light.h"
#include "nvs_flash.h"

/*******************************************************
 *                Macros
 *******************************************************/

#define WIFI_TX_PWR     60  //LOOK BELOW
// 8 2dbm,20,28,34,44,52,56,60,66,72,80 20dbm.

/*******************************************************
 *                Constants
 *******************************************************/
#define RX_SIZE          (1500)
#define TX_SIZE          (1460)

/*******************************************************
 *                Variable Definitions
 *******************************************************/
static const char *MESH_TAG = "mesh_main";
static const uint8_t MESH_ID[6] = { 0x77, 0x77, 0x77, 0x77, 0x77, 0x77};
static uint8_t tx_buf[TX_SIZE] = { 0, };
static uint8_t rx_buf[RX_SIZE] = { 0, };
static bool is_running = true;
static bool is_mesh_connected = false;
static mesh_addr_t mesh_parent_addr;
static int mesh_layer = -1;
static esp_netif_t *netif_sta = NULL;

//#pragma region COMMON

#define MAC_MATCH(x, y) ((x[0] == y[0]) && (x[1] == y[1]) && (x[2] == y[2]) && (x[3] == y[3]) && (x[4] == y[4]) && (x[5] == y[5]))
#define MAC6_SIZE (6)

#define MAC_CODE(x) ((((uint16_t)x[4]) << 8) | x[5])

#define PKT_ID_COUNTER 0x3A
#define PKT_ID_COUNTER_ANS  0xCA

typedef struct payload_head_ {
    uint32_t ssc;
    uint8_t pkt_id;
} payload_head_t;

typedef struct payload_node_ {
    payload_head_t head;
    //Spefic payload
    uint8_t layer;              //Mesh library obtained layer
    uint8_t parent[MAC6_SIZE];  //
    int8_t rssi;                //RSSI of the node to the first parent
} payload_node_t;

typedef struct payload_root_ {
    payload_head_t head;
    //Spefic payload
    int8_t txPwr;   //TX Power to set
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
static int8_t _wifi_tx_pwr = 0;
static uint32_t _ssc_sent = 0;

/*******************************************************
 *                Function Definitions
 *******************************************************/

void mesh_root_draw_network(){
    static uint64_t lastDrawnUS = 0; 
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
}

void esp_mesh_p2p_tx_main(void *arg)
{
    int i;
    esp_err_t err;
    int send_count = 0;
    mesh_addr_t route_table[CONFIG_MESH_ROUTE_TABLE_SIZE];
    int route_table_size = 0;
    mesh_data_t data;
    data.data = tx_buf;
    data.size = sizeof(tx_buf);
    data.proto = MESH_PROTO_BIN;
    data.tos = MESH_TOS_P2P;
    is_running = true;

    payload_root_t *pt;
    rtt_t * r;

    while (is_running) {
        /* non-root do nothing but print */
        if (!esp_mesh_is_root()) {
            // We are a node, therefore we just send replies to the root
            ESP_LOGI(MESH_TAG, "layer:%d, rtableSize:%d, %s", mesh_layer,
                     esp_mesh_get_routing_table_size(),
                     (is_mesh_connected && esp_mesh_is_root()) ? "ROOT" : is_mesh_connected ? "NODE" : "DISCONNECT");
            vTaskDelay(10 * 1000 / portTICK_RATE_MS);
            continue;
        }

        // ROOT handling
        esp_mesh_get_routing_table((mesh_addr_t *) &route_table,
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
            
            err = esp_mesh_send(&route_table[i], &data, MESH_DATA_P2P, NULL, 0);
            if (err) {
                ESP_LOGE(MESH_TAG,
                         "[ROOT-2-UNICAST:%d][L:%d]parent:"MACSTR" to "MACSTR", heap:%d[err:0x%x, proto:%d, tos:%d]",
                         send_count, mesh_layer, MAC2STR(mesh_parent_addr.addr),
                         MAC2STR(route_table[i].addr), esp_get_minimum_free_heap_size(),
                         err, data.proto, data.tos);
            } else {
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

        mesh_root_draw_network();

        /* if route_table_size is less than 10, add delay to avoid watchdog in this task. */
        //if (route_table_size < 10) {
            vTaskDelay(1 * 1000 / portTICK_RATE_MS);
        //}
    }
    vTaskDelete(NULL);
}

void esp_mesh_p2p_rx_main(void *arg)
{
    int recv_count = 0;
    esp_err_t err;
    mesh_addr_t from;
    mesh_data_t data;
    int flag = 0;
    data.data = rx_buf;
    data.size = RX_SIZE;
    is_running = true;

    while (is_running) {
        data.size = RX_SIZE;
        err = esp_mesh_recv(&from, &data, portMAX_DELAY, &flag, NULL, 0);
        if (err != ESP_OK || !data.size) {
            ESP_LOGE(MESH_TAG, "err:0x%x, size:%d", err, data.size);
            continue;
        }

        if (MAC_MATCH(_my_macSTA, from.addr)) {
            //Let's ignore ourself
            continue;
        }

        recv_count++;

        if (!esp_mesh_is_root()) {
            wifi_ap_record_t apr;
            payload_root_t *pr;
            payload_node_t *pt;
            if (data.size < sizeof(payload_root_t)) {
                ESP_LOGW(MESH_TAG, "Packet too short! %d", data.size);
                continue;
            }

            pr = (payload_root_t *)data.data;
            if (pr->head.pkt_id != PKT_ID_COUNTER) {
                ESP_LOGW(MESH_TAG, "Received unexpected packet %02X", pr->head.pkt_id);
                continue;
            }

            uint32_t ssc = ntohl(pr->head.ssc);
            int8_t txpwr = pr->txPwr;
            ESP_LOGI(MESH_TAG, "RECV SSC %d from " MACSTR, ssc, MAC2STR(from.addr));

            //Use payload
            if (txpwr != _wifi_tx_pwr) {
                //Change TX power as requested from root
                ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(txpwr));
                _wifi_tx_pwr = txpwr;
            }
            //End

            pt = (payload_node_t *)data.data;
            pt->head.pkt_id = PKT_ID_COUNTER_ANS;
            pt->head.ssc = htonl(ssc);
            pt->layer = (uint8_t)mesh_layer;
            memcpy(pt->parent, mesh_parent_addr.addr, MAC6_SIZE);
            ESP_ERROR_CHECK(esp_wifi_sta_get_ap_info(&apr)); //Result also RSSI data
            pt->rssi = apr.rssi;

            data.size = sizeof(payload_node_t);

            //Answer back to the root
            err = esp_mesh_send(&from, &data, MESH_DATA_P2P, NULL, 0);
            if (err != ESP_OK)
            {
                ESP_LOGE(MESH_TAG, "TX err:0x%x", err);
            }
        }
        else
        {
            // ROOT Handling   
            payload_node_t *p;
            rtt_t* r;

            /* extract send count */
            if (data.size < sizeof(payload_node_t))
            {
                ESP_LOGW(MESH_TAG, "Packet too short from %02X%02X! %d",
                         from.addr[4],
                         from.addr[5],
                         data.size);
                continue;
            }

            p = (payload_node_t *)data.data;
            if (p->head.pkt_id != PKT_ID_COUNTER_ANS)
            {
                ESP_LOGW(MESH_TAG, "Received unexpected packet %02X from %02X%02X",
                         p->head.pkt_id,
                         from.addr[4],
                         from.addr[5]);
                continue;
            }

            int ssc = ntohl(p->head.ssc);
            for (int i = 0; i < MAX_RTT_SENT; ++i)
            {
                r = &_rounders[i];
                if (r->id == ssc)
                {
                    ESP_LOGI(MESH_TAG, "%X: RTT %d. Elapsed %dms",
                             MAC_CODE(from.addr),
                             r->id,
                             (uint32_t)((esp_timer_get_time() - r->start) / 1000));
                    r->used = false;
                    break;
                }
            }

            //Go on with the packet
            for (int i = 0; i < MAX_DEVICES; ++i)
            {
                if (_my_network[i].exists && _my_network[i].mac_coded == MAC_CODE(from.addr))
                {
                    //Update parent and layer info
                    _my_network[i].layer = p->layer;
                    _my_network[i].parent_coded = MAC_CODE(p->parent);
                    _my_network[i].rssi = p->rssi;
                }
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
        ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_AP, _my_macAP));

        xTaskCreate(esp_mesh_p2p_tx_main, "MPTX", 3072, NULL, 5, NULL);
        xTaskCreate(esp_mesh_p2p_rx_main, "MPRX", 3072, NULL, 5, NULL);
    }
    return ESP_OK;
}

void mesh_root_routing_table_add()
{
    mesh_addr_t route_table[CONFIG_MESH_ROUTE_TABLE_SIZE];
    int route_table_size = 0;

    // Since the event does not give a list of the new devices we have to check ALL of them :(
    esp_mesh_get_routing_table((mesh_addr_t *)&route_table,
                               CONFIG_MESH_ROUTE_TABLE_SIZE * 6, &route_table_size);

    for (int j = 0; j < route_table_size; ++j) {
        // Add node, if needed
        int i;
        for (i = 0; i < MAX_DEVICES; ++i) {
            // We don't trust callback to be called only for new nodes.
            // If we already have the device in the map, we use that one
            if (_my_network[i].mac_coded == MAC_CODE(route_table[j].addr)) {
                _my_network[i].exists = true; // Entry is active from here
                break;
            }
        }
        // If the device mac was not found in the list
        if (i >= MAX_DEVICES) {
            for (i = 0; i < MAX_DEVICES; ++i) {
                // Get the first free slot and set it up
                if (!_my_network[i].exists)  {
                    // Create entry
                    _my_network[i].mac_coded = MAC_CODE(route_table[j].addr);
                    _my_network[i].layer = 0;
                    _my_network[i].exists = true; //Entry is active from here
                    break;
                }
            }
        }
    }
}

void mesh_root_routing_table_remove() {
    mesh_addr_t route_table[CONFIG_MESH_ROUTE_TABLE_SIZE];
    int route_table_size = 0;

    // Since the event does not give a list of the removed devices we have to check ALL of them :(
    esp_mesh_get_routing_table((mesh_addr_t *)&route_table,
                                CONFIG_MESH_ROUTE_TABLE_SIZE * 6, &route_table_size);

    bool found = false;

    for (int i = 0; i < MAX_DEVICES; ++i) {
        // Remove any device that is not referenced in the routing table anymore
        found = false;

        if (!_my_network[i].exists)
            continue; // Skip inactive slots

        // Search for the mac in the routing table
        for (int j = 0; j < route_table_size; ++j) {
            if (_my_network[i].mac_coded == MAC_CODE(route_table[j].addr))
            {
                found = true;
                break; // uniqueness is guaranteed in the add function
            }
        }
        if (found) {
            // Reset the slot
            _my_network[i].exists = false;
            _my_network[i].mac_coded = 0;
        }
    }
}

void mesh_event_handler(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data)
{
    mesh_addr_t id = {0,};
    static uint16_t last_layer = 0;

    switch (event_id) {
    case MESH_EVENT_STARTED: {
        esp_mesh_get_id(&id);
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_MESH_STARTED>ID:"MACSTR"", MAC2STR(id.addr));
        is_mesh_connected = false;
        mesh_layer = esp_mesh_get_layer();
    }
    break;
    case MESH_EVENT_STOPPED: {
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_STOPPED>");
        is_mesh_connected = false;
        mesh_layer = esp_mesh_get_layer();
    }
    break;
    case MESH_EVENT_CHILD_CONNECTED: {
        mesh_event_child_connected_t *child_connected = (mesh_event_child_connected_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_CHILD_CONNECTED>aid:%d, "MACSTR"",
                 child_connected->aid,
                 MAC2STR(child_connected->mac));
    }
    break;
    case MESH_EVENT_CHILD_DISCONNECTED: {
        mesh_event_child_disconnected_t *child_disconnected = (mesh_event_child_disconnected_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_CHILD_DISCONNECTED>aid:%d, "MACSTR"",
                 child_disconnected->aid,
                 MAC2STR(child_disconnected->mac));
    }
    break;
    case MESH_EVENT_ROUTING_TABLE_ADD: {
        mesh_event_routing_table_change_t *routing_table = (mesh_event_routing_table_change_t *)event_data;
        ESP_LOGW(MESH_TAG, "<MESH_EVENT_ROUTING_TABLE_ADD>add %d, new:%d, layer:%d",
                 routing_table->rt_size_change,
                 routing_table->rt_size_new, mesh_layer);

        if (esp_mesh_is_root()) {
            mesh_root_routing_table_add();
        }
    }
    break;
    case MESH_EVENT_ROUTING_TABLE_REMOVE: {
        mesh_event_routing_table_change_t *routing_table = (mesh_event_routing_table_change_t *)event_data;
        ESP_LOGW(MESH_TAG, "<MESH_EVENT_ROUTING_TABLE_REMOVE>remove %d, new:%d, layer:%d",
                 routing_table->rt_size_change,
                 routing_table->rt_size_new, mesh_layer);

        if (esp_mesh_is_root()) {
            mesh_root_routing_table_remove();
        }

    }
    break;
    case MESH_EVENT_NO_PARENT_FOUND: {
        mesh_event_no_parent_found_t *no_parent = (mesh_event_no_parent_found_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_NO_PARENT_FOUND>scan times:%d",
                 no_parent->scan_times);
    }
    /* TODO handler for the failure */
    break;
    case MESH_EVENT_PARENT_CONNECTED: {
        mesh_event_connected_t *connected = (mesh_event_connected_t *)event_data;
        esp_mesh_get_id(&id);
        mesh_layer = connected->self_layer;
        memcpy(&mesh_parent_addr.addr, connected->connected.bssid, 6);
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_PARENT_CONNECTED>layer:%d-->%d, parent:"MACSTR"%s, ID:"MACSTR", duty:%d",
                 last_layer, mesh_layer, MAC2STR(mesh_parent_addr.addr),
                 esp_mesh_is_root() ? "<ROOT>" :
                 (mesh_layer == 2) ? "<layer2>" : "", MAC2STR(id.addr), connected->duty);
        last_layer = mesh_layer;
        mesh_connected_indicator(mesh_layer);
        is_mesh_connected = true;
        if (esp_mesh_is_root()) {
            esp_netif_dhcpc_stop(netif_sta);
            esp_netif_dhcpc_start(netif_sta);
        }
        esp_mesh_comm_p2p_start();
    }
    break;
    case MESH_EVENT_PARENT_DISCONNECTED: {
        mesh_event_disconnected_t *disconnected = (mesh_event_disconnected_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_PARENT_DISCONNECTED>reason:%d",
                 disconnected->reason);
        is_mesh_connected = false;
        mesh_disconnected_indicator();
        mesh_layer = esp_mesh_get_layer();
    }
    break;
    case MESH_EVENT_LAYER_CHANGE: {
        mesh_event_layer_change_t *layer_change = (mesh_event_layer_change_t *)event_data;
        mesh_layer = layer_change->new_layer;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_LAYER_CHANGE>layer:%d-->%d%s",
                 last_layer, mesh_layer,
                 esp_mesh_is_root() ? "<ROOT>" :
                 (mesh_layer == 2) ? "<layer2>" : "");
        last_layer = mesh_layer;
        mesh_connected_indicator(mesh_layer);
    }
    break;
    case MESH_EVENT_ROOT_ADDRESS: {
        mesh_event_root_address_t *root_addr = (mesh_event_root_address_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_ADDRESS>root address:"MACSTR"",
                 MAC2STR(root_addr->addr));
    }
    break;
    case MESH_EVENT_VOTE_STARTED: {
        mesh_event_vote_started_t *vote_started = (mesh_event_vote_started_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_VOTE_STARTED>attempts:%d, reason:%d, rc_addr:"MACSTR"",
                 vote_started->attempts,
                 vote_started->reason,
                 MAC2STR(vote_started->rc_addr.addr));
    }
    break;
    case MESH_EVENT_VOTE_STOPPED: {
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_VOTE_STOPPED>");
        break;
    }
    case MESH_EVENT_ROOT_SWITCH_REQ: {
        mesh_event_root_switch_req_t *switch_req = (mesh_event_root_switch_req_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_ROOT_SWITCH_REQ>reason:%d, rc_addr:"MACSTR"",
                 switch_req->reason,
                 MAC2STR( switch_req->rc_addr.addr));
    }
    break;
    case MESH_EVENT_ROOT_SWITCH_ACK: {
        /* new root */
        mesh_layer = esp_mesh_get_layer();
        esp_mesh_get_parent_bssid(&mesh_parent_addr);
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_SWITCH_ACK>layer:%d, parent:"MACSTR"", mesh_layer, MAC2STR(mesh_parent_addr.addr));
    }
    break;
    case MESH_EVENT_TODS_STATE: {
        mesh_event_toDS_state_t *toDs_state = (mesh_event_toDS_state_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_TODS_REACHABLE>state:%d", *toDs_state);
    }
    break;
    case MESH_EVENT_ROOT_FIXED: {
        mesh_event_root_fixed_t *root_fixed = (mesh_event_root_fixed_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_FIXED>%s",
                 root_fixed->is_fixed ? "fixed" : "not fixed");
    }
    break;
    case MESH_EVENT_ROOT_ASKED_YIELD: {
        mesh_event_root_conflict_t *root_conflict = (mesh_event_root_conflict_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_ROOT_ASKED_YIELD>"MACSTR", rssi:%d, capacity:%d",
                 MAC2STR(root_conflict->addr),
                 root_conflict->rssi,
                 root_conflict->capacity);
    }
    break;
    case MESH_EVENT_CHANNEL_SWITCH: {
        mesh_event_channel_switch_t *channel_switch = (mesh_event_channel_switch_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_CHANNEL_SWITCH>new channel:%d", channel_switch->channel);
    }
    break;
    case MESH_EVENT_SCAN_DONE: {
        mesh_event_scan_done_t *scan_done = (mesh_event_scan_done_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_SCAN_DONE>number:%d",
                 scan_done->number);
    }
    break;
    case MESH_EVENT_NETWORK_STATE: {
        mesh_event_network_state_t *network_state = (mesh_event_network_state_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_NETWORK_STATE>is_rootless:%d",
                 network_state->is_rootless);
    }
    break;
    case MESH_EVENT_STOP_RECONNECTION: {
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_STOP_RECONNECTION>");
    }
    break;
    case MESH_EVENT_FIND_NETWORK: {
        mesh_event_find_network_t *find_network = (mesh_event_find_network_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_FIND_NETWORK>new channel:%d, router BSSID:"MACSTR"",
                 find_network->channel, MAC2STR(find_network->router_bssid));
    }
    break;
    case MESH_EVENT_ROUTER_SWITCH: {
        mesh_event_router_switch_t *router_switch = (mesh_event_router_switch_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROUTER_SWITCH>new router:%s, channel:%d, "MACSTR"",
                 router_switch->ssid, router_switch->channel, MAC2STR(router_switch->bssid));
    }
    break;
    case MESH_EVENT_PS_PARENT_DUTY: {
        mesh_event_ps_duty_t *ps_duty = (mesh_event_ps_duty_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_PS_PARENT_DUTY>duty:%d", ps_duty->duty);
    }
    break;
    case MESH_EVENT_PS_CHILD_DUTY: {
        mesh_event_ps_duty_t *ps_duty = (mesh_event_ps_duty_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_PS_CHILD_DUTY>cidx:%d, "MACSTR", duty:%d", ps_duty->child_connected.aid-1,
                MAC2STR(ps_duty->child_connected.mac), ps_duty->duty);
    }
    break;
    default:
        ESP_LOGI(MESH_TAG, "unknown id:%d", event_id);
        break;
    }
}

void ip_event_handler(void *arg, esp_event_base_t event_base,
                      int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    ESP_LOGI(MESH_TAG, "<IP_EVENT_STA_GOT_IP>IP:" IPSTR, IP2STR(&event->ip_info.ip));

}

void app_main(void)
{
    ESP_ERROR_CHECK(mesh_light_init());
    ESP_ERROR_CHECK(nvs_flash_init());
    /*  tcpip initialization */
    ESP_ERROR_CHECK(esp_netif_init());
    /*  event initialization */
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    /*  create network interfaces for mesh (only station instance saved for further manipulation, soft AP instance ignored */
    ESP_ERROR_CHECK(esp_netif_create_default_wifi_mesh_netifs(&netif_sta, NULL));
    /*  wifi initialization */
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    ESP_ERROR_CHECK(esp_wifi_start());

    //Set custom TX power
    int8_t pwr;
    ESP_ERROR_CHECK(esp_wifi_get_max_tx_power(&pwr));
    ESP_LOGI(MESH_TAG, "WIFI power currently set to %d.", pwr);
    pwr = WIFI_TX_PWR;
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(pwr));

    /*  mesh initialization */
    ESP_ERROR_CHECK(esp_mesh_init());
    ESP_ERROR_CHECK(esp_event_handler_register(MESH_EVENT, ESP_EVENT_ANY_ID, &mesh_event_handler, NULL));
    /*  set mesh topology */
    ESP_ERROR_CHECK(esp_mesh_set_topology(CONFIG_MESH_TOPOLOGY));
    /*  set mesh max layer according to the topology */
    ESP_ERROR_CHECK(esp_mesh_set_max_layer(CONFIG_MESH_MAX_LAYER));
    ESP_ERROR_CHECK(esp_mesh_set_vote_percentage(1));
    ESP_ERROR_CHECK(esp_mesh_set_xon_qsize(128));
#ifdef CONFIG_MESH_ENABLE_PS
    /* Enable mesh PS function */
    ESP_ERROR_CHECK(esp_mesh_enable_ps());
    /* better to increase the associate expired time, if a small duty cycle is set. */
    ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(60));
    /* better to increase the announce interval to avoid too much management traffic, if a small duty cycle is set. */
    ESP_ERROR_CHECK(esp_mesh_set_announce_interval(600, 3300));
#else
    /* Disable mesh PS function */
    ESP_ERROR_CHECK(esp_mesh_disable_ps());
    ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(10));
#endif
    mesh_cfg_t cfg = MESH_INIT_CONFIG_DEFAULT();
    /* mesh ID */
    memcpy((uint8_t *) &cfg.mesh_id, MESH_ID, 6);
    /* router */
    cfg.channel = CONFIG_MESH_CHANNEL;
    cfg.router.ssid_len = strlen(CONFIG_MESH_ROUTER_SSID);
    memcpy((uint8_t *) &cfg.router.ssid, CONFIG_MESH_ROUTER_SSID, cfg.router.ssid_len);
    memcpy((uint8_t *) &cfg.router.password, CONFIG_MESH_ROUTER_PASSWD,
           strlen(CONFIG_MESH_ROUTER_PASSWD));
    /* mesh softAP */
    ESP_ERROR_CHECK(esp_mesh_set_ap_authmode(CONFIG_MESH_AP_AUTHMODE));
    cfg.mesh_ap.max_connection = CONFIG_MESH_AP_CONNECTIONS;
    cfg.mesh_ap.nonmesh_max_connection = CONFIG_MESH_NON_MESH_AP_CONNECTIONS;
    memcpy((uint8_t *) &cfg.mesh_ap.password, CONFIG_MESH_AP_PASSWD,
           strlen(CONFIG_MESH_AP_PASSWD));
    ESP_ERROR_CHECK(esp_mesh_set_config(&cfg));
    /* mesh start */
    ESP_ERROR_CHECK(esp_mesh_start());
#ifdef CONFIG_MESH_ENABLE_PS
    /* set the device active duty cycle. (default:10, MESH_PS_DEVICE_DUTY_REQUEST) */
    ESP_ERROR_CHECK(esp_mesh_set_active_duty_cycle(CONFIG_MESH_PS_DEV_DUTY, CONFIG_MESH_PS_DEV_DUTY_TYPE));
    /* set the network active duty cycle. (default:10, -1, MESH_PS_NETWORK_DUTY_APPLIED_ENTIRE) */
    ESP_ERROR_CHECK(esp_mesh_set_network_duty_cycle(CONFIG_MESH_PS_NWK_DUTY, CONFIG_MESH_PS_NWK_DUTY_DURATION, CONFIG_MESH_PS_NWK_DUTY_RULE));
#endif
    ESP_LOGI(MESH_TAG, "mesh starts successfully, heap:%d, %s<%d>%s, ps:%d\n",  esp_get_minimum_free_heap_size(),
             esp_mesh_is_root_fixed() ? "root fixed" : "root not fixed",
             esp_mesh_get_topology(), esp_mesh_get_topology() ? "(chain)":"(tree)", esp_mesh_is_ps_enabled());
}
