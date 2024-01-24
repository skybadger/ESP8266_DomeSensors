<h2>ESP8266 Dome Sensors</h2>
This application runs on the ESP8266-01 wifi-enabled SoC device to capture sensor readings and transmit them to the local MQTT service. 
In my arrangement, Node-red flows are used to listen for and graph the updated readings in the dashboard UI. 
The unit is setup for I2C operation and is expecting to see SCL on GIO0 and SDA on GPIO2 with the one-wire interface to the DHT sensor on GPIO3. 
The pressure sensor is the BMP280. If that is not present, the unit will continually reboot. 

<h2>Dependencies:</h2>
<ul>
  <li>Arduino 1.6, </li>
<li>ESP8266 V2.4+ </li>
<li>Arduino MQTT client (https://pubsubclient.knolleary.net/api.html)</li>
<li>Arduino DHT111 sensor library (https://github.com/beegee-tokyo/DHTesp/archive/master.zip)</li>
<li>Arduino JSON library (pre v6) </li>
<li>BMP280 library (https://github.com/orgua/iLib )</li></ul>

<h3>Testing</h3>
Access by serial port  - Tx only is available from device at 115,600 baud at 3.3v. THis provides debug output .
Wifi is used for MQTT reporting only and servicing web requests
Use http://ESPTHM01 to receive json-formatted output of current sensors. 

<h3>Use:</h3>
I use mine to source a dashboard via hosting the mqtt server in node-red. It runs off a solar-panel supply in my observatory dome. 

<h3>ToDo:</h3>
Add a HMC5883 compass interface 
