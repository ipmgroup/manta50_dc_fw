import dronecan
import time
import re

node = dronecan.make_node("can0", node_id=111)
node_monitor = dronecan.app.node_monitor.NodeMonitor(node)

param_index = {
    "NODE_ID": (0, int),
    "ESC_INDEX": (1, int),
    "ARMING": (2, int),
    "TELEM_RATE": (3, int),
    "CAN_SPEED": (4, int),
    "MAX_SPEED": (5, float),
    "CONTROL_WORD": (6, int),
    "MIDLE_POINT": (7, float),
    "ACCELER": (8, float),
    "MOTOR_POLES": (9, int),
    "KP": (10, float),
    "KI": (11, float),
    "RES_EST_CURRENT": (12, float),
    "IND_EST_CURRENT": (13, float),
    "MAX_CURRENT": (14, float),
    "FLUX_EST_FREQ": (15, float),
    "Rs": (16, float),
    "Ld": (17, float),
    "FLUX": (18, float),
}

# column_index = param_index.get(param_name, [None])[0]
current_index = 0
REQUEST_PRIORITY = 30
nodeid = 6

dic_param = {}


def empty_callback(event):
    pass


def send_request(index):
    request = dronecan.uavcan.protocol.param.GetSet.Request(index=index)
    node.request(
        request, nodeid, empty_callback, priority=REQUEST_PRIORITY, timeout=3000.0
    )


def send_next_request():
    if current_index < len(param_index):
        send_request(current_index)
        current_index += 1
    else:
        current_index = 0


def handle_debug_log_message(event):
    global dic_param
    message = event.transfer.payload
    try:
        decoded_text = message.text.decode("utf-8")
        items = decoded_text.split(" ")
        i = 0
        while i < len(items):
            key = items[i]
            values = items[i + 1 : i + 4]
            dic_param[key] = values
            i += 1 + len(values)
    except Exception as e:
        print(f"Error processing message: {e}")


node.add_handler(dronecan.uavcan.protocol.debug.LogMessage, handle_debug_log_message)


def set_parameter_value(index, value):
    request_payload = dronecan.uavcan.protocol.param.GetSet.Request(
        index=index,
        value=dronecan.uavcan.protocol.param.Value(integer_value=value, timeout=5000.0),
    )
    node.request(request_payload, 6, empty_callback)


try:
    node.spin(timeout=1.5)
    print("\nDetected nodes")
    for id in sorted(list(node_monitor.get_all_node_id())):
        d = node_monitor.get(id)
        name = str(d.info.name) if not isinstance(d.info.name, str) else d.info.name
        if re.search(r"org\.ipm\.manta", name):
            nodeid = d.node_id
            print("  ", nodeid, ":", name)
    # node.spin(timeout=1.5)

    while True:
        node.spin(1.5)

        for index in param_index.values():
            ix, itype = index
            send_request(int(ix))
            node.spin(timeout=1.5)

        if len(dic_param) == len(param_index):
            break
        else:
            time.sleep(0.5)

    print("\nNode parameters:")
    for key, values in dic_param.items():
        values_str = ", ".join(values)
        print(f"{key}: {values_str}")

except KeyboardInterrupt:

    node.close()
    print("Node stopped.")
