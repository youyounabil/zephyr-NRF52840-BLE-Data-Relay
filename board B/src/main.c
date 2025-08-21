#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <dk_buttons_and_leds.h>
#include <string.h>

#define RUN_STATUS_LED DK_LED1
#define CONNECTION_STATUS_LED DK_LED2
#define RUN_LED_BLINK_INTERVAL 500

static struct bt_conn *my_conn = NULL;
char received_data[100] = {0};
static bool new_data_flag = false; // flag for new data

LOG_MODULE_REGISTER(peripheral_app, LOG_LEVEL_INF);

// ---------------- CUSTOM UUIDs ----------------
// Custom Service UUID: 12345678-1234-5678-1234-123456789abc
/* Custom Service UUID: 12345678-1234-5678-1234-123456789abc */
static struct bt_uuid_128 custom_service_uuid = BT_UUID_INIT_128(
	0xbc, 0x9a, 0x78, 0x56,
	0x34, 0x12, 0x78, 0x56,
	0x34, 0x12, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12
);

/* Custom Characteristic UUID: abcdefab-cdef-4567-89ab-cdef12345678 */
static struct bt_uuid_128 custom_char_uuid = BT_UUID_INIT_128(
   0x78, 0x56, 0x34, 0x12,
    0xef, 0xcd, 0xab, 0x89,
    0x67, 0x45, 0xef, 0xcd,
    0xab, 0xef, 0xcd, 0xab
);


// ---------------- GATT HANDLERS ----------------
static ssize_t write_data(struct bt_conn *conn,
                          const struct bt_gatt_attr *attr,
                          const void *buf, uint16_t len,
                          uint16_t offset, uint8_t flags)
{
    uint16_t copy_len = MIN(len, sizeof(received_data) - 1);

    memcpy(received_data, buf, copy_len);
    received_data[copy_len] = '\0'; // null terminate

    new_data_flag = true; // mark new data arrived

    return len;
}

BT_GATT_SERVICE_DEFINE(custom_svc,
    BT_GATT_PRIMARY_SERVICE(&custom_service_uuid),
    BT_GATT_CHARACTERISTIC(&custom_char_uuid.uuid,
                           BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE,
                           NULL, write_data, NULL),
);

// ---------------- ADVERTISING ----------------
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, "Nordic_Peripheral", 17),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL,
                  0x9c,0xab,0x78,0x56,0x34,0x12,0x34,0x12,
                  0x78,0x56,0x34,0x12,0x78,0x56,0x34,0x12)
};

// ---------------- CONNECTION HANDLERS ----------------
void on_connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection error %d", err);
        return;
    }

    LOG_INF("Connected");
    my_conn = bt_conn_ref(conn);
    dk_set_led(CONNECTION_STATUS_LED, 1);
}

void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected. Reason %d", reason);
    if (my_conn) {
        bt_conn_unref(my_conn);
        my_conn = NULL;
    }
    dk_set_led(CONNECTION_STATUS_LED, 0);
}

struct bt_conn_cb conn_callbacks = {
    .connected = on_connected,
    .disconnected = on_disconnected,
};

// ---------------- MAIN ----------------
int main(void)
{
    int blink_status = 0;
    int err;

    LOG_INF("Starting Peripheral Example");

    err = dk_leds_init();
    if (err) {
        LOG_ERR("LEDs init failed (err %d)", err);
        return -1;
    }

    bt_conn_cb_register(&conn_callbacks);

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return -1;
    }

    LOG_INF("Bluetooth initialized");

    static const struct bt_le_adv_param adv_param = {
        .options = BT_LE_ADV_OPT_CONNECTABLE,
        .interval_min = BT_GAP_ADV_FAST_INT_MIN_1,
        .interval_max = BT_GAP_ADV_FAST_INT_MAX_1,
        .id = BT_ID_DEFAULT,
    };

    err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return -1;
    }

    LOG_INF("Advertising successfully started");

    for (;;) {
        // Blink RUN LED
        dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);

        // Always check for new data
        if (new_data_flag) {
            printk("📩 New data received from central: %s\n", received_data);
            new_data_flag = false; // reset flag
        }

        k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
    }
}
