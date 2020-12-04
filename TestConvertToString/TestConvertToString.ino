void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  double a = 10.2010;

  String SerialData="";

  SerialData = String(a,4);

  Serial.println(SerialData);
}
