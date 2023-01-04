import paho.mqtt.client as mqtt
import sys

def on_connect(client, userdata, flags, rc):
    client.subscribe("#")


def send_led_command(r, g, b):
    client.publish("face_light/color", bytes([r,g,b]))

client = mqtt.Client()
client.on_connect = on_connect
client.connect("192.168.123.161",1883,60)
send_led_command(0,0,0)
client.loop_forever()