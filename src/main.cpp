#include "Arduino.h"
#include "ESP8266WiFi.h"
#include "PubSubClient.h"
#include "ArduinoJson.h"

#include <config.h>

#define MOTOR_LEFT_PWM_PIN D1
#define MOTOR_RIGHT_PWM_PIN D2
#define MOTOR_LEFT_DIR_PIN D3
#define MOTOR_RIGHT_DIR_PIN D4

#define MOTOR_DEADZONE 150

double P_TURN = 10;
double I_TURN = 0;
double D_TURN = 1;
double TURN_INTEGRAL, TURN_LAST_ERROR = 0;

double P_DRIVE = 20;
double I_DRIVE = 1;
double D_DRIVE = 0;
double DRIVE_INTEGRAL, DRIVE_LAST_ERROR = 0;

double TARGET_X = 0;
double TARGET_Y = 0.5;

uint32_t LAST_LOOP = 0;

char buffer[2048];

bool AT_TARGET = false;

StaticJsonDocument<2048> MQTT_DOC;
StaticJsonDocument<2048> SERIAL_DOC;
StaticJsonDocument<4096> LOG_DOC;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

void connectWifi(String SSID, String Password)
{
    delay(1000);
    WiFi.begin(SSID, Password);
    int tries = 1;
    Serial.println(SSID);
    Serial.println(Password);
    Serial.println("Connecting");

    while (WiFi.status() != WL_CONNECTED)
    {

        Serial.print(".");
        tries++;

        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);
    }
    Serial.println("Connected");
}

void connectMQTT(String ClientName, String User, String Password)
{
    mqttClient.setServer(CONF_MQTT_SERVER, CONF_MQTT_PORT);
    while (!mqttClient.connected())
    {
        Serial.println("Connecting to mqtt");
        digitalWrite(LED_BUILTIN, HIGH);

        if (mqttClient.connect(ClientName.c_str(), User.c_str(), Password.c_str()))
        {
            Serial.println("connected");
        }
        else
        {
            Serial.print("failed With state: ");
            Serial.print(mqttClient.state());
            delay(1000);
        }
    }
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
    Serial.println("MQTT received");
    deserializeJson(MQTT_DOC, payload);
    serializeJsonPretty(MQTT_DOC, Serial);

    if (MQTT_DOC["tpid"])
    {
        P_TURN = int(MQTT_DOC["tpid"]["p"]);
        I_TURN = int(MQTT_DOC["tpid"]["i"]);
        D_TURN = int(MQTT_DOC["tpid"]["d"]);

        TURN_INTEGRAL = 0;
    }
    if (MQTT_DOC["dpid"])
    {
        P_DRIVE = int(MQTT_DOC["dpid"]["p"]);
        I_DRIVE = int(MQTT_DOC["dpid"]["i"]);
        D_DRIVE = int(MQTT_DOC["dpid"]["d"]);

        DRIVE_INTEGRAL = 0;
    }
    if (MQTT_DOC["target"])
    {
        TARGET_X = float(MQTT_DOC["target"]["x"]);
        TARGET_Y = float(MQTT_DOC["target"]["y"]);

        DRIVE_INTEGRAL = 0;
    }
}

void write_motors(int drive, int turn)
{

    int rightOutput = constrain(drive - turn, -512, 512); // they should be devided by sqrt(2) to be mathematically correct
    int leftOutput = constrain(turn + drive, -512, 512);  // it currently both rotates and scales the coordinate system, but i dont care

    LOG_DOC["outputs"]["rawLeft"] = leftOutput;
    LOG_DOC["outputs"]["rawRight"] = rightOutput;

    int mappedRightOutput = map(abs(rightOutput), 0, 512, MOTOR_DEADZONE, 512);
    int mappedLeftOutput = map(abs(leftOutput), 0, 512, MOTOR_DEADZONE, 512);

    mappedRightOutput = rightOutput < 0 ? -mappedRightOutput : mappedRightOutput;
    mappedLeftOutput = leftOutput < 0 ? -mappedLeftOutput : mappedLeftOutput;

    LOG_DOC["outputs"]["left"] = mappedLeftOutput;
    LOG_DOC["outputs"]["right"] = mappedRightOutput;

    analogWrite(MOTOR_LEFT_PWM_PIN, abs(mappedLeftOutput));
    analogWrite(MOTOR_RIGHT_PWM_PIN, abs(mappedRightOutput));

    digitalWrite(MOTOR_LEFT_DIR_PIN, mappedLeftOutput < 0);
    digitalWrite(MOTOR_RIGHT_DIR_PIN, mappedRightOutput < 0);
}

long int pid(int current, int target, uint32_t timeSinceLast, double P, double I, double D, double &integral, double &lastError)
{
    int error = current - target;
    integral += error * (timeSinceLast / 1000000.0);
    double derivative = (error - lastError) / (timeSinceLast / 1000000.0);
    long int output = (P * error + I * integral + D * derivative);

    LOG_DOC["pid"]["current"] = current;
    LOG_DOC["pid"]["taget"] = target;
    LOG_DOC["pid"]["output"] = output;
    LOG_DOC["pid"]["p"] = error * P;
    LOG_DOC["pid"]["I"] = integral * I;
    LOG_DOC["pid"]["D"] = derivative * D;

    lastError = error;
    return output;
}

void setup()
{
    Serial.begin(115200);
    Serial.setTimeout(1000);

    pinMode(D1, OUTPUT);
    pinMode(D2, OUTPUT);
    pinMode(D3, OUTPUT);
    pinMode(D4, OUTPUT);

    analogWriteRange(512);

    connectWifi(CONF_SSID, CONF_PASSWORD);
    connectMQTT(CONF_MQTT_CLIENT_NAME, CONF_MQTT_USER, CONF_MQTT_PASSWORD);

    mqttClient.setCallback(mqttCallback);
    mqttClient.publish(CONF_MQTT_SUB_TOPIC, "hello from esp");
    mqttClient.subscribe(CONF_MQTT_SUB_TOPIC);
}

void loop()
{
    if (Serial.available())
    {
        DeserializationError err = deserializeJson(SERIAL_DOC, Serial);
        LOG_DOC.clear();
        if (err.code() == DeserializationError::Ok && !SERIAL_DOC["position"].isNull() && !SERIAL_DOC["rotation"].isNull())
        {
            mqttClient.loop();

            float dx = float(SERIAL_DOC["position"]["x"]) - TARGET_X;
            float dy = float(SERIAL_DOC["position"]["z"]) - TARGET_Y;

            LOG_DOC["phone"]["rot"] = SERIAL_DOC["rotation"];
            LOG_DOC["phone"]["pos"] = SERIAL_DOC["position"];

            LOG_DOC["target"]["x"] = TARGET_X;
            LOG_DOC["target"]["y"] = TARGET_Y;

            float phone_angle = float(SERIAL_DOC["rotation"]["y"]);
            float angle;

            if (dy < 0)
            {
                angle = atan(abs(dx) / abs(dy));
            }
            else
            {
                angle = atan(abs(dy) / abs(dx)) + PI / 2;
            }

            if (dx > 0)
            {
                angle *= -1;
            }

            angle = angle * 180 / PI;
            phone_angle = phone_angle - 360 * (phone_angle > 180);

            int error = (int(angle - phone_angle + 540.0) % 360) - 180;
            int output = pid(error, 0, micros() - LAST_LOOP, P_TURN, I_TURN, D_TURN, TURN_INTEGRAL, TURN_LAST_ERROR);

            float driveVal = 0;

            LOG_DOC["should_drive"] = abs(angle - phone_angle) < 6;
            LOG_DOC["angle_diff"] = abs(angle - phone_angle);
            LOG_DOC["pos_diff"] = sqrt(dx * dx + dy * dy);
            LOG_DOC["at_target"] = sqrt(dx * dx + dy * dy) < 0.02;
            LOG_DOC["outputs"]["drive"] = driveVal;

            if (abs(angle - phone_angle) < 6)
            {
                if (sqrt(dx * dx + dy * dy) > 0.02)
                {
                    driveVal = sqrt(dx * dx + dy * dy) * P_DRIVE;
                    LOG_DOC["outputs"]["drive"] = driveVal;
                }
                else
                {
                    write_motors(0, 0);
                    return;
                }
            }

            write_motors(constrain(-driveVal, -512, 512), constrain(output, -512, 512));

            LOG_DOC["outputs"]["error"] = error;
            LOG_DOC["outputs"]["output"] = output;
            LOG_DOC["outputs"]["phone_angle"] = phone_angle;
            LOG_DOC["outputs"]["angle"] = angle;

            LAST_LOOP = micros();

            size_t characters = serializeJsonPretty(LOG_DOC, buffer);

            mqttClient.beginPublish(CONF_MQTT_PUB_TOPIC, characters, false);
            for (size_t i = 0; i < characters; i++)
            {
                mqttClient.write(buffer[i]);
            }
            mqttClient.endPublish();

            mqttClient.publish(CONF_MQTT_PUB_TOPIC, buffer);
            // mqttClient.publish(CONF_MQTT_PUB_TOPIC, "hej\n");

            /*
            int length = Serial.available();
            mqttClient.beginPublish(CONF_MQTT_PUB_TOPIC, length, false);

            for (size_t i = 0; i < length; i++)
            {
                mqttClient.write(Serial.read());
            }
            */
        }
        else
        {
            mqttClient.publish(CONF_MQTT_PUB_TOPIC, err.c_str());
        }
    }
    // write_motors(constrain((int((float) SERIAL_DOC["position"]["x"] * 1000) + (int(MQTT_DOC["drive"]["forward"]) - int(MQTT_DOC["drive"]["backward"]))) * 20 , -512, 512) ,0);
}
// write_motors(constrain((int((float) SERIAL_DOC["position"]["x"] * 1000) + (int(MQTT_DOC["drive"]["forward"]) - int(MQTT_DOC["drive"]["backward"]))) * 20 , -512, 512) ,0);