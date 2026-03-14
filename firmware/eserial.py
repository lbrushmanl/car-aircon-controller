import serial
import serial.tools.list_ports
from enum import IntEnum
import construct as cn


class EnumDisplayWrapper:
    def __init__(self, enum_member):
        self.enum_member = enum_member

    def __getattr__(self, item):
        return getattr(self.enum_member, item)

    def __repr__(self):
        return self.enum_member.name

    def __str__(self):
        return self.enum_member.name

    def __eq__(self, other):
        if isinstance(other, EnumDisplayWrapper):
            return self.enum_member == other.enum_member
        return self.enum_member == other

    def __hash__(self):
        return hash(self.enum_member)


class MessageIds(IntEnum):
    AIRCON_STATE = 10
    DATA = 11
    TEST_RELAYS = 12
    GET_CONFIGURATION_DATA = 13
    SET_CONFIGURATION_DATA = 14


class Bool(IntEnum):
    FALSE = 0
    TRUE = 1


class AirConStates(IntEnum):
    OFF = 0
    ON = 1
    COOLDOWN = 2


class PersistentDataIds(IntEnum):
    SAFE_RUN_TIME = 0
    COOL_DOWN_TIME = 1
    BATTERY_FILTER_ALPHA = 2
    DISPLAY_SLEEP_TIMER = 3


class QAdapter(cn.Adapter):
    def __init__(self, subcon, q_value):
        super().__init__(subcon)
        self.q_value = q_value

    def _decode(self, obj, context, path):
        # Convert integer to float
        return obj / (1 << self.q_value)

    def _encode(self, obj, context, path):
        # Convert float back to integer
        return int(round(obj * (1 << self.q_value)))


class EnumAdapter(cn.Adapter):
    def __init__(self, subcon, enum):
        super().__init__(subcon)
        self.enum = enum

    def _decode(self, obj, context, path):
        return EnumDisplayWrapper(self.enum(obj))

    def _encode(self, obj, context, path):
        return int(obj)


class BoolAdapter(cn.Adapter):
    def __init__(self, subcon):
        super().__init__(subcon)

    def _decode(self, obj, context, path):
        return bool(obj)

    def _encode(self, obj, context, path):
        return int(obj)


Q8 = QAdapter(cn.Int32sl, q_value=8)


payload_wrapper = cn.Struct("size" / cn.Int8ul, "payload" / cn.Bytes(cn.this.size))

tmp = cn.Struct(
    "setpoint" / Q8,
    "temperature" / Q8,
    "battery_voltage" / Q8,
)

aircon_state = cn.Struct(
    "enabled" / BoolAdapter(cn.Int8ul),
    "running" / BoolAdapter(cn.Int8ul),
    "state" / EnumAdapter(cn.Int8ul, AirConStates),
    "run_time" / cn.Int16ul,
)


persistent_data_payloads = {
    PersistentDataIds.SAFE_RUN_TIME: cn.Int16ul,
    PersistentDataIds.COOL_DOWN_TIME: cn.Int16ul,
    PersistentDataIds.BATTERY_FILTER_ALPHA: cn.Int16ul,
    PersistentDataIds.DISPLAY_SLEEP_TIMER: cn.Int8ul,
}

persistent_data = cn.Struct(
    "persistent_data_id" / EnumAdapter(cn.Int8ul, PersistentDataIds),
    "payload" / cn.Switch(cn.this.persistent_data_id, persistent_data_payloads),
)

message_responses = {
    MessageIds.AIRCON_STATE: aircon_state,
    MessageIds.DATA: tmp,
    MessageIds.GET_CONFIGURATION_DATA: persistent_data,
    MessageIds.SET_CONFIGURATION_DATA: persistent_data,
}


message_requests = {
    MessageIds.AIRCON_STATE: cn.Pass,
    MessageIds.DATA: cn.Pass,
    MessageIds.GET_CONFIGURATION_DATA: persistent_data,
    MessageIds.SET_CONFIGURATION_DATA: persistent_data,
}


message_response = cn.Struct(
    "message_id" / EnumAdapter(cn.Int8ul, MessageIds),
    "payload" / cn.Switch(cn.this.message_id, message_responses),
)


message_request = cn.Struct(
    "message_id" / EnumAdapter(cn.Int8ul, MessageIds),
    "payload" / cn.Switch(cn.this.message_id, message_requests),
)


serial_ports = [comport.device for comport in serial.tools.list_ports.comports()]
print(serial_ports)

ser = serial.Serial(serial_ports[0], timeout=0.1)
while True:
    msg_select = input()
    request = None
    if msg_select[0] == "a":
        request = {"message_id": MessageIds.AIRCON_STATE, "payload": cn.Pass}
    elif msg_select[0] == "d":
        request = {"message_id": MessageIds.DATA, "payload": cn.Pass}
    elif msg_select[0] == "g":
        read_data = {
            "persistent_data_id": int(msg_select[1]),
            "payload": 0,
        }

        request = {
            "message_id": MessageIds.GET_CONFIGURATION_DATA,
            "payload": read_data,
        }
    elif msg_select[0] == "s":
        set_data = {
            "persistent_data_id": int(msg_select[1]),
            "payload": int(msg_select[2]),
        }

        request = {"message_id": MessageIds.SET_CONFIGURATION_DATA, "payload": set_data}

    if request:
        msg_request = message_request.build(request)
        payload = {"size": len(msg_request), "payload": msg_request}
        data = payload_wrapper.build(payload)
        ser.write(data)
        raw_bytes = ser.read(32)
        raw_msg = payload_wrapper.parse(raw_bytes)
        parsed_msg = message_response.parse(raw_msg.payload)
        print(parsed_msg)
