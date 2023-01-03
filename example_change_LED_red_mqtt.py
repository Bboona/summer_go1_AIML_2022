import paho.mqtt.client as mqtt
import sys

def on_connect(client, userdata, flags, rc):
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("#")


def send_led_command(client, r, g, b):
    command = f"child_conn.send('change_light({r},{g},{b})')"
    client.publish("face_light/color", bytes([r,g,b]))

client = mqtt.Client()
client.on_connect = on_connect
client.connect("192.168.123.161",1883,60)
send_led_command(client, 255, 0, 0)
client.loop_forever()