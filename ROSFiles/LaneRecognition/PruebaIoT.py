import time

from paho.mqtt import client as mqtt_client


port = 1883
topic = "/ITESM/PUEBLA/jetracer"
ClientID = "Test"

def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(ClientID)
    client.on_connect = on_connect
    client.connect("localhost", port, 60)
    return client


def publish(client):
    while True:
        msg_count = 255
        #msg = f"messages: {msg_count}"
        result = client.publish(topic, msg_count)
        # result: [0, 1]
        status = result[0]
        if status == 0:
            print(f"Send `{msg_count}` to topic `{topic}`")
        else:
            print(f"Failed to send message to topic {topic}")

        time.sleep(5)

def run():
    client = connect_mqtt()
    client.loop_start()
    publish(client)


if __name__ == '__main__':
    run()