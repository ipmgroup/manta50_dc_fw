import dronecan
import time
import re
import json
import argparse
import sys

# Создаем узел и монитор
node = dronecan.make_node("can0", node_id=111)
node_monitor = dronecan.app.node_monitor.NodeMonitor(node)

# Индексы параметров
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

REQUEST_PRIORITY = 30
dic_param = {}
send_param = {}


def empty_callback(event):
    pass


def send_request(index):
    request = dronecan.uavcan.protocol.param.GetSet.Request(index=index)
    node.request(request, nodeid, empty_callback, priority=REQUEST_PRIORITY, timeout=3000.0)


def handle_debug_log_message(event):
    global dic_param
    message = event.transfer.payload
    try:
        decoded_text = message.text.decode("utf-8")
        items = decoded_text.split(" ")
        i = 0
        while i < len(items):
            key = items[0]
            values = items[i + 1 : i + 4]
            dic_param[key] = values
            i += 1 + len(values)
    except Exception as e:
        print(f"Error processing message: {e}")


node.add_handler(dronecan.uavcan.protocol.debug.LogMessage, handle_debug_log_message)


def send_value(index, value):
    if isinstance(value, float):
        request = dronecan.uavcan.protocol.param.GetSet.Request(
            index=index,
            value=dronecan.uavcan.protocol.param.Value(real_value=value),
        )
    else:
        request = dronecan.uavcan.protocol.param.GetSet.Request(
            index=index,
            value=dronecan.uavcan.protocol.param.Value(integer_value=value),
        )

    node.request(request, nodeid, empty_callback, timeout=5000.0)


def save_parameters_to_file(filename="parameters.json"):
    try:
        with open(filename, "w") as f:
            json.dump(dic_param, f, indent=4)
        print(f"Parameters saved to {filename}")
    except Exception as e:
        print(f"Error saving parameters to file: {e}")


def load_parameters_from_file(filename="parameters.json"):
    global send_param
    try:
        with open(filename, "r") as f:
            send_param = json.load(f)
        print(f"Parameters loaded from {filename}")
    except Exception as e:
        print(f"Error loading parameters from file: {e}")


def parse_arguments():
    parser = argparse.ArgumentParser(description="DroneCAN Parameter Manager")
    parser.add_argument(
        "--mode",
        choices=["read", "write"],
        help="Operation mode: 'read' to read parameters or 'write' to set parameters.",
    )
    parser.add_argument("--node_id", type=int, help="Node ID to communicate with.")
    parser.add_argument("--set_node_id", type=int, help="New Node ID to set in parameters.")
    parser.add_argument("--esc_index", type=int, help="ESC index to set in parameters.")
    parser.add_argument(
        "--filename",
        type=str,
        default="parameters.json",
        help="Filename to save/load parameters.",
    )
    return parser.parse_args()


def print_progress_bar(iteration, total, length=50):
    percent = (iteration / total) * 100
    filled_length = int(length * iteration // total)
    bar = "█" * filled_length + "-" * (length - filled_length)
    sys.stdout.write(f"\rProgress: |{bar}| {percent:.2f}%")
    sys.stdout.flush()


try:
    args = parse_arguments()
    if args.mode is None:
        args.mode = "read"

    if args.node_id is not None:
        nodeid = args.node_id
    else:
        node.spin(timeout=1.5)
        print("\nDetected nodes")
        for id in sorted(list(node_monitor.get_all_node_id())):
            d = node_monitor.get(id)
            name = str(d.info.name) if not isinstance(d.info.name, str) else d.info.name
            if re.search(r"org\.ipm\.manta", name):
                nodeid = d.node_id
                print("  ", nodeid, ":", name)

    if args.mode == "write":
        if args.esc_index is None and args.set_node_id is None:
            print("Error: --esc_index or --set_node_id must be specified for write mode.")
            exit(1)

        load_parameters_from_file(args.filename)

        if args.set_node_id is not None:
            send_param["NODE_ID"] = [str(args.set_node_id)]

        if args.esc_index is not None:
            send_param["ESC_INDEX"] = [str(args.esc_index)]

        while True:
            verification_failed = False

            diff_dict = {key: send_param[key] for key in send_param if key not in dic_param}

            if not diff_dict:
                print("All parameters are sent.")
                break

            for key, value in diff_dict.items():
                if key == "MIDLE_POINT":
                    parts = send_param["MIDLE_POINT"]
                    midle_point_value = 1 if int(parts[0]) else 0
                    esc_min_value = int(parts[1]) & 0x3FF
                    esc_max_value = int(parts[2]) & 0x1FFF
                    packed_int = (midle_point_value << 31) | (esc_min_value << 21) | (esc_max_value << 8)

                    send_value(param_index["MIDLE_POINT"][0], packed_int)
                else:
                    param_type = param_index[key][1]
                    value_to_send = param_type(send_param[key][0])

                    send_value(param_index[key][0], value_to_send)

                node.spin(timeout=0.1)

            # time.sleep(0.5)

        exit(0)

    if args.mode == "read":
        total_parameters = len(param_index)
        read_count = 0

        while True:
            diff_dict = {
                key: param_index[key] for key in param_index if key not in dic_param or dic_param[key] != param_index[key]
            }
            node.spin(1.5)
            for key, value in diff_dict.items():
                send_request(int(value[0]))
                node.spin(timeout=1.5)

                read_count = len(dic_param)
                print_progress_bar(read_count, total_parameters)

            if len(dic_param) == total_parameters:
                break
            else:
                time.sleep(0.5)

        print("\n\nNode parameters:")
        for key, values in dic_param.items():
            values_str = ", ".join(values)
            print(f"{key}: {values_str}")

        save_parameters_to_file(args.filename)
        print("\nReading complete.")

except KeyboardInterrupt:
    node.close()
    print("Node stopped.")
