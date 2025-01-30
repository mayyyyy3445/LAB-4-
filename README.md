# LAB-4-

This is a circuit diagram featuring an Arduino Uno connected to a breadboard, which includes an LED and what appears to be a temperature sensor (likely a TMP36 or LM35).

Breakdown of the Components and Connections:
	1.	Arduino Uno:
	•	This is the microcontroller board that processes input signals and controls output devices.
	2.	LED with a Resistor:
	•	The LED is placed on the breadboard and is connected in series with a resistor (likely 220Ω or 330Ω) to limit the current.
	•	One side of the resistor is connected to ground (GND) of the Arduino.
	•	The other leg of the LED is connected to an Arduino digital pin (possibly pin 9 or 10) through a wire (orange in color), which means the Arduino controls when the LED turns on/off.
	3.	Temperature Sensor (Likely TMP36 or LM35):
	•	The three-pin component on the breadboard appears to be a temperature sensor.
	•	The connections are:
	•	VCC (Power) Pin: Connected to the 5V pin of the Arduino.
	•	GND Pin: Connected to the GND of the Arduino.
	•	Output Pin: Connected to one of the analog input pins (A0 or A1) of the Arduino.

Functionality of the Circuit:
	•	The temperature sensor reads the ambient temperature and sends an analog signal to the Arduino.
	•	The Arduino processes this data and can take an action, such as:
	•	Turning ON the LED if the temperature exceeds a certain threshold.
	•	Keeping the LED OFF if the temperature is below the threshold.


// Define pins
const int sensorPin = A0;  // TMP36/LM35 output connected to A0
const int ledPin = 9;      // LED connected to digital pin 9

void setup() {
  Serial.begin(9600);      // Start serial communication
  pinMode(ledPin, OUTPUT); // Set LED pin as output
}

void loop() {
  int sensorValue = analogRead(sensorPin); // Read analog voltage from sensor
  float voltage = sensorValue * (5.0 / 1023.0); // Convert to voltage (0-5V)
  
  // TMP36: Convert voltage to temperature (°C)
  float temperatureC = (voltage - 0.5) * 100.0; // TMP36: 10mV per °C

  Serial.print("Temperature: ");
  Serial.print(temperatureC);
  Serial.println(" °C");

  // If temperature exceeds 30°C, turn on LED
  if (temperatureC > 30.0) {
    digitalWrite(ledPin, HIGH); // Turn LED ON
  } else {
    digitalWrite(ledPin, LOW);  // Turn LED OFF
  }

  delay(1000); // Wait 1 second before next reading
}
