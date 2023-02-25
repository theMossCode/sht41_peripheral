/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(MAIN);

#define MAIN_SERVICE_UUID	BT_UUID_128_ENCODE(0xedd1a5f3, 0xdbb0, 0x4b29, 0xb449, 0xa4be5161f18e)
#define RX_UUID				BT_UUID_128_ENCODE(0xedd1a5f3, 0xdbb2, 0x4b29, 0xb449, 0xa4be5161f18e)
#define TX_UUID				BT_UUID_128_ENCODE(0xedd1a5f3, 0xdbb3, 0x4b29, 0xb449, 0xa4be5161f18e)

#define MAIN_EVT_TIMER_EXPIRY			0x01
#define MAIN_EVT_BLE_RESP_RECEIVED		0x02

#define TIMER_INTERVAL_MINUTES			1

#define BLE_PASSKEY						123456

#define SHT41_NODE						DT_NODELABEL(sht41)

struct sht41_data{
	double temp;
	double rh;
};

static ssize_t rx_chr_written(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags);
static ssize_t tx_chr_read_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset);
static void tx_chr_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);

static void connected_cb(struct bt_conn *conn, uint8_t err);
static void disconnected_cb(struct bt_conn *conn, uint8_t reason);
static void security_changed_cb(struct bt_conn *conn, bt_security_t level, enum bt_security_err err);

static void pair_cancel(struct bt_conn *conn);
static void pairing_confirm(struct bt_conn *conn);

static void pairing_complete(struct bt_conn *conn, bool bonded);
static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason);
static void bond_deleted(uint8_t id, const bt_addr_le_t *peer);

static void sensor_timer_expiry_handler(struct k_timer *timer);

const struct bt_uuid_128 main_service_uuid = BT_UUID_INIT_128(MAIN_SERVICE_UUID);
const struct bt_uuid_128 tx_uuid = BT_UUID_INIT_128(TX_UUID);
const struct bt_uuid_128 rx_uuid = BT_UUID_INIT_128(RX_UUID);
struct bt_conn *conn;

struct bt_conn_cb conn_callbacks = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
	.security_changed = security_changed_cb
};
struct bt_conn_auth_cb conn_auth_callbacks = {
	.cancel = pair_cancel,
	.pairing_confirm = pairing_confirm
};
struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed,
	.bond_deleted = bond_deleted
};

bool notifications_enabled = false;

const struct device *sht41 = DEVICE_DT_GET(SHT41_NODE);
struct sht41_data sht41_sensor_data;

// Sensor timer definition
K_TIMER_DEFINE(sensor_timer, sensor_timer_expiry_handler, NULL);

// Events
K_EVENT_DEFINE(main_evts);

static const struct bt_data adv_data [] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, MAIN_SERVICE_UUID)
};

BT_GATT_SERVICE_DEFINE(primary_service, 
	BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(MAIN_SERVICE_UUID)),
	BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(RX_UUID), BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, rx_chr_written, NULL),
	BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(TX_UUID), BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, tx_chr_read_cb, NULL, NULL),
	BT_GATT_CCC(tx_chr_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static ssize_t rx_chr_written(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	LOG_DBG("Data Received");
	const uint8_t *data = (char *) buf;
	if(data[0] == 0x00){
		LOG_INF("Response received");
		k_event_set(&main_evts, MAIN_EVT_BLE_RESP_RECEIVED);
	}
	else if(data[0] == 0x01){
		// retry
		LOG_DBG("Retry");
		k_timer_start(&sensor_timer, K_SECONDS(15), K_SECONDS(15));
	}
	else{
		LOG_DBG("Unexpected response %d", data[0]);
	}

	return len;
}

static ssize_t tx_chr_read_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
	LOG_DBG("TX read");
	return 0;
}

static void tx_chr_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	if(value){
		notifications_enabled = true;
		// start timer
		k_timer_start(&sensor_timer, K_MINUTES(TIMER_INTERVAL_MINUTES), K_MINUTES(TIMER_INTERVAL_MINUTES));
		LOG_INF("TX notifications enabled");
	}
	else{
		notifications_enabled = false;
		// stop timer to conserve power
		k_timer_stop(&sensor_timer);
		LOG_WRN("TX notifications disabled");
	}
}

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	LOG_INF("Device connected, %d", err);
	bt_conn_ref(conn);
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	LOG_WRN("Device disconnected %d", reason);
	bt_conn_unref(conn);
}

static void security_changed_cb(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	if(err){
		LOG_ERR("Updated security error %d", err);
		return;
	}

	LOG_DBG("Security updated to %d", level);
}

static void pair_cancel(struct bt_conn *conn)
{
	// Not used
	LOG_DBG("Cancel pairing");
}

static void pairing_confirm(struct bt_conn *conn)
{
	if(bt_conn_auth_pairing_confirm(conn)){
		LOG_ERR("Confirm pairing error");
		return;
	}

	LOG_DBG("Pairing confirm");
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	LOG_INF("Pairing complete");
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	LOG_ERR("Pairing fail, reason %d", reason);
}

static void bond_deleted(uint8_t id, const bt_addr_le_t *peer)
{
	LOG_DBG("Bond info deleted!");
}

static void sensor_timer_expiry_handler(struct k_timer *timer)
{
	// set timer expiry event
	k_event_set(&main_evts, MAIN_EVT_TIMER_EXPIRY);
}

static int ble_init()
{
	int ret = 0;
	ret = bt_enable(NULL);
	if(ret){
		LOG_ERR("Bluetooth enable error %d", ret);
		return ret;
	}

	bt_conn_cb_register(&conn_callbacks);

	ret = bt_conn_auth_cb_register(&conn_auth_callbacks);
	if(ret){
		LOG_ERR("Authentication callbacks register fail");
		return ret;
	}

	ret = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
	if(ret){
		LOG_ERR("Authentication info callbacks register fail");
		return ret;
	}
	LOG_DBG("Callbacks registered");

	ret = bt_passkey_set(BLE_PASSKEY);
	if(ret){
		LOG_ERR("Set fixed passkey fail");
		return ret;
	}

	ret = bt_le_adv_start(BT_LE_ADV_CONN_NAME, adv_data, ARRAY_SIZE(adv_data), NULL, 0);
	if(ret){
		LOG_ERR("Error starting advertising");
		return ret;
	}

	LOG_INF("BLE advertising started");

	return 0;
}

static int sht41_fetch_data()
{
	int ret = 0;
	struct sensor_value t;
	struct sensor_value rh;
	ret = sensor_sample_fetch(sht41);
	if(ret){
		LOG_DBG("Sample fetch error");
		return ret;
	}

	ret = sensor_channel_get(sht41, SENSOR_CHAN_AMBIENT_TEMP, &t);
	if(ret){
		LOG_ERR("Error getting ambient temperature");
		return ret;
	}

	ret = sensor_channel_get(sht41, SENSOR_CHAN_HUMIDITY, &rh);
	if(ret){
		LOG_ERR("Error getting relative humidity");
		return ret;
	}

	sht41_sensor_data.temp = sensor_value_to_double(&t);
	sht41_sensor_data.rh = sensor_value_to_double(&rh);

	return 0;
}

static int sht41_init()
{
	if(!device_is_ready(sht41)){
		LOG_ERR("%s device not ready", sht41->name);
		return -ENODEV;
	}

	return 0;
}

void main(void)
{
	int res = 0;
	uint32_t event;
	int16_t notify_buffer[2];
	if(ble_init()){
		return;
	}

	if(sht41_init()){
		return;
	}

	while(1){
		event = k_event_wait(&main_evts, MAIN_EVT_TIMER_EXPIRY, true, K_FOREVER);
		if(!(event & MAIN_EVT_TIMER_EXPIRY)){
			LOG_WRN("Unexpected event %d", event);
			continue;
		}

		res = sht41_fetch_data();
		if(res){
			LOG_ERR("Error %d fetching sensor data", res);
			// restart timer with shorter duration
			k_timer_start(&sensor_timer, K_SECONDS(15), K_SECONDS(15));
			continue;
		}

		if(notifications_enabled){
			notify_buffer[0] = (int16_t)(sht41_sensor_data.temp * 100);
			notify_buffer[1] = (int16_t)(sht41_sensor_data.rh * 100);

			res = bt_gatt_notify(conn, &primary_service.attrs[3], (const void *)notify_buffer, sizeof(notify_buffer));
			if(res){
				LOG_WRN("Notify error %d", res);
				k_timer_start(&sensor_timer, K_SECONDS(15), K_SECONDS(15));				
				continue;
			}
		}

		event = k_event_wait(&main_evts, MAIN_EVT_BLE_RESP_RECEIVED, true, K_SECONDS(5));
		if(!(event & MAIN_EVT_BLE_RESP_RECEIVED)){
			LOG_WRN("BLE wait resp timeout");
			k_timer_start(&sensor_timer, K_SECONDS(15), K_SECONDS(15));
			continue;
		}

		// reset timer
		k_timer_start(&sensor_timer, K_MINUTES(TIMER_INTERVAL_MINUTES), K_MINUTES(TIMER_INTERVAL_MINUTES));
	}
}
