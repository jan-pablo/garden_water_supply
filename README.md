# garden_water_supply

Goal:
- Control of two water pipelines. Water can be send to either one or both water lines. 
- Water flow rate is determind and used for S- and S+ control as well as to provide data for calculating water consumption (daily, weekly, monthly, yearly).
- Send all values (status and flow rates, summed volumes) to FHEM-OS by MQTT.
- Accepting commands via MQTT for setting valves - connected to FHEM and FHEM-Tablet-UI  
- Timer for autmatic closure of valves after a defined time
- Messenger which is sending a reminder to my mobile every 30 Minutes if the irrigation is active
- Pre-Alarm messages to my mobile if temperature, humidity in the moisture-proof box (containing WEMOS, Trafo, Relais) is increased but not at critical values
- Safety measures which closes all valves if temperature or humidity in the moisture-proof box (containing WEMOS, Trafo, Relais) is exceeding critical values + alarm message to my mobile
- Safety measures which closes all valves if flow control indicates hole rupture or blockage (by too high or to low water flow) + alarm message to my mobile >>>not activated yet<<<

Description of equipment and wiring:
- 1 Hall flow meter for water flow measurement
- 3 Magnet-Valves for control of flow-direction - powered by one 12V Trafo via a 4 channel relay
- 4 LED for indication of flow-direction and status - powered by one 12V Trafo via a 4 channel relay
- 3 5V Relais for contolling the 12V Magnet Valves - powered by one 12V Trafo
- BME280 for measuring temperature, pressure, humidity within the Moisture-proof box in which the whole equipment (except the valves and the hall flow sensor) is protected from rain and weather  
- WEMOS D1 Mini for program execution, MQTT communication, sensor input and contolling relais - powered by WEMOS Battery shield 

Wiring of equipment:
- 5 V to USB of Wemos-Battery shield
- 12 V to actor-side of relais (12V ground to magnetic valves)
- Relais N1,N2,N3,N4 input to: Wemos D3, D6, D4, D8  (GPIO: 0, 12, 2, 15)
- BME280: VIN: 3.3V; GND: ground;SCL: D1(GPIO: 5);SDA: D2(GPIO: 4)
- Flow Sensor: D7(GPIO: 13), VIN: 3.3V; GND: ground
- LED:
    - green: on when relai is on - wired in parellel to the magnetic valves
    - red: on when Wemos is online - short blink during MQTT data transfer    >>>not activated yet<<<
