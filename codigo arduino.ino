int x1=1;
int y1=1;

int x2=1;
int y2=1;

int x3=1;
int y3=1;
String rec;
void setup() {
  Serial.begin(9600);
}

void loop() {

    y1=analogRead(0);
    y2=analogRead(2);
    y3=analogRead(4);
    x1=analogRead(1);
    x2=analogRead(3);
    x3=analogRead(5);
    Serial.println((String)x1+","+y1+","+x2+","+y2+","+x3+","+y3);
    

}
